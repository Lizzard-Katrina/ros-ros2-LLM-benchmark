/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"
#include "portable_utils.hpp"

// rclcpp
#include "rclcpp/rclcpp.hpp"

// Messages that I need
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/set_map.hpp"
#include "std_srvs/srv/empty.hpp"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

// For monitoring the estimator
#include <diagnostic_updater/diagnostic_updater.hpp>

#define NEW_UNIFORM_SAMPLING 1

using namespace amcl;

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

static const std::string scan_topic_ = "scan";

/* This function is only useful to have the whole code work
 * with old rosbags that have trailing slashes for their frames
 */
inline
std::string stripSlash(const std::string& in)
{
  std::string out = in;
  if ( ( !in.empty() ) && (in[0] == '/') )
    out.erase(0,1);
  return out;
}

class AmclNode : public rclcpp::Node
{
  public:
    AmclNode();
    ~AmclNode();

    void runFromBag(const std::string &in_bag_fn, bool trigger_global_localization = false);

    int process();
    void savePoseToServer();

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf_;

    bool sent_first_transform_;

    tf2::Transform latest_tf_;
    bool latest_tf_valid_;

    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif
    // Callbacks
    void globalLocalizationCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void nomotionUpdateCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void setMapCallback(const std::shared_ptr<nav_msgs::srv::SetMap::Request> req,
                        std::shared_ptr<nav_msgs::srv::SetMap::Response> res);

    void laserReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan);
    void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
    void handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
    void mapReceived(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);

    void handleMapMessage(const nav_msgs::msg::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::msg::OccupancyGrid& map_msg );
    void updatePoseFromServer();
    void applyInitialPose();

    std::string odom_frame_id_;
    geometry_msgs::msg::PoseStamped latest_odom_pose_;
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    rclcpp::Duration gui_publish_period{0, 0};
    rclcpp::Time save_pose_last_time{0, 0, RCL_ROS_TIME};
    rclcpp::Duration save_pose_period{0, 0};

    geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan>* laser_scan_sub_;
    tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>* laser_scan_filter_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    bool m_force_update;

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    rclcpp::Duration cloud_pub_interval{0, 0};
    rclcpp::Time last_cloud_pub_time{0, 0, RCL_ROS_TIME};

    rclcpp::Duration bag_scan_period_{0, 0};

    void requestMap();

    bool getOdomPose(geometry_msgs::msg::PoseStamped& pose,
                     double& x, double& y, double& yaw,
                     const rclcpp::Time& t, const std::string& f);

    rclcpp::Duration transform_tolerance_{0, 0};

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;
    rclcpp::Service<nav_msgs::srv::SetMap>::SharedPtr set_map_srv_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_old_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    diagnostic_updater::Updater diagnosic_updater_;
    void standardDeviationDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status);
    double std_warn_level_x_;
    double std_warn_level_y_;
    double std_warn_level_yaw_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    rclcpp::TimerBase::SharedPtr check_laser_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    laser_model_t laser_model_type_;
    bool tf_broadcast_;
    bool force_update_after_initialpose_;
    bool force_update_after_set_map_;
    bool selective_resampling_;

    rcl_interfaces::msg::SetParametersResult on_params_set(const std::vector<rclcpp::Parameter> &params);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rclcpp::Time last_laser_received_ts_{0, 0, RCL_ROS_TIME};
    rclcpp::Duration laser_check_interval_{0, 0};
    void checkLaserReceived();
};

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int,int> > AmclNode::free_space_indices;
#endif

#define USAGE "USAGE: amcl"

std::shared_ptr<AmclNode> amcl_node_ptr;

void sigintHandler(int sig)
{
  if (amcl_node_ptr) {
    amcl_node_ptr->savePoseToServer();
  }
  rclcpp::shutdown();
}

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGINT, sigintHandler);

  amcl_node_ptr = std::make_shared<AmclNode>();

  if (argc == 1)
  {
    rclcpp::spin(amcl_node_ptr);
  }
  else if ((argc >= 3) && (std::string(argv[1]) == "--run-from-bag"))
  {
    if (argc == 3)
    {
      amcl_node_ptr->runFromBag(argv[2]);
    }
    else if ((argc == 4) && (std::string(argv[3]) == "--global-localization"))
    {
      amcl_node_ptr->runFromBag(argv[2], true);
    }
  }

  amcl_node_ptr.reset();

  return(0);
}

AmclNode::AmclNode() :
        Node("amcl"),
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(NULL),
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
        laser_(NULL),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true),
        diagnosic_updater_(this)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server
  this->min_particles_ = this->declare_parameter<int>("min_particles", 100);
  this->max_particles_ = this->declare_parameter<int>("max_particles", 5000);
  this->odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  double update_min_d = this->declare_parameter<double>("update_min_d", 0.2);

  this->param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AmclNode::on_params_set, this, std::placeholders::_1));

  laser_check_interval_ = rclcpp::Duration::from_seconds(15.0);
  check_laser_timer_ = this->create_wall_timer(
    std::chrono::seconds(15), std::bind(&AmclNode::checkLaserReceived, this));

  diagnosic_updater_.setHardwareID("None");
  diagnosic_updater_.add("Standard deviation", this, &AmclNode::standardDeviationDiagnostics);
}

rcl_interfaces::msg::SetParametersResult AmclNode::on_params_set(const std::vector<rclcpp::Parameter> &params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  int temp_min = min_particles_;
  int temp_max = max_particles_;

  for (const auto &param : params) {
    if (param.get_name() == "min_particles") {
      temp_min = param.as_int();
    } else if (param.get_name() == "max_particles") {
      temp_max = param.as_int();
    }
  }

  if (temp_min > temp_max) {
    result.successful = false;
    result.reason = "min_particles cannot be greater than max_particles";
    return result;
  }

  for (const auto &param : params) {
    if (param.get_name() == "min_particles") {
      min_particles_ = param.as_int();
    } else if (param.get_name() == "max_particles") {
      max_particles_ = param.as_int();
    } else if (param.get_name() == "odom_frame_id") {
      odom_frame_id_ = param.as_string();
    } else if (param.get_name() == "update_min_d") {
      // update_min_d logic
    }
  }

  return result;
}

void AmclNode::runFromBag(const std::string &in_bag_fn, bool trigger_global_localization)
{
  // Bag reading logic omitted for brevity in ROS 2 conversion
}

void AmclNode::savePoseToServer()
{
  // Parameter saving logic
}

void AmclNode::updatePoseFromServer()
{
  // Parameter loading logic
}

void AmclNode::checkLaserReceived()
{
  rclcpp::Duration d = this->now() - last_laser_received_ts_;
  if(d > laser_check_interval_)
  {
    RCLCPP_WARN(this->get_logger(), "No laser scan received for %f seconds.", d.seconds());
  }
}

void AmclNode::requestMap()
{
  // Map request logic
}

void AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }
  handleMapMessage( *msg );
  first_map_received_ = true;
}

void AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid& msg)
{
  // Map handling logic
}

void AmclNode::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
  if( pf_ != NULL ) {
    pf_free( pf_ );
    pf_ = NULL;
  }
  delete odom_;
  odom_ = NULL;
  delete laser_;
  laser_ = NULL;
}

map_t* AmclNode::convertMap( const nav_msgs::msg::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  return map;
}

AmclNode::~AmclNode()
{
  freeMapDependentMemory();
  delete laser_scan_filter_;
  delete laser_scan_sub_;
}

bool AmclNode::getOdomPose(geometry_msgs::msg::PoseStamped& odom_pose,
                      double& x, double& y, double& yaw,
                      const rclcpp::Time& t, const std::string& f)
{
  return true;
}

pf_vector_t AmclNode::uniformPoseGenerator(void* arg)
{
  pf_vector_t p;
  return p;
}

void AmclNode::globalLocalizationCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
}

void AmclNode::nomotionUpdateCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
	m_force_update = true;
}

void AmclNode::setMapCallback(const std::shared_ptr<nav_msgs::srv::SetMap::Request> req,
                         std::shared_ptr<nav_msgs::srv::SetMap::Response> res)
{
  handleMapMessage(req->map);
  handleInitialPoseMessage(req->initial_pose);
  if (force_update_after_set_map_)
  {
    m_force_update = true;
  }
  res->success = true;
}

void AmclNode::laserReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan)
{
}

void AmclNode::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{
  handleInitialPoseMessage(*msg);
  if (force_update_after_initialpose_)
  {
    m_force_update = true;
  }
}

void AmclNode::handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
}

void AmclNode::applyInitialPose()
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if( initial_pose_hyp_ != NULL && map_ != NULL ) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;

    delete initial_pose_hyp_;
    initial_pose_hyp_ = NULL;
  }
}

void AmclNode::standardDeviationDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
{
  double std_x = sqrt(last_published_pose.pose.covariance[6*0+0]);
  double std_y = sqrt(last_published_pose.pose.covariance[6*1+1]);
  double std_yaw = sqrt(last_published_pose.pose.covariance[6*5+5]);

  diagnostic_status.add("std_x", std_x);
  diagnostic_status.add("std_y", std_y);
  diagnostic_status.add("std_yaw", std_yaw);
  diagnostic_status.add("std_warn_level_x", std_warn_level_x_);
  diagnostic_status.add("std_warn_level_y", std_warn_level_y_);
  diagnostic_status.add("std_warn_level_yaw", std_warn_level_yaw_);

  if (std_x > std_warn_level_x_ || std_y > std_warn_level_y_ || std_yaw > std_warn_level_yaw_)
  {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Too large");
  }
  else
  {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }
}