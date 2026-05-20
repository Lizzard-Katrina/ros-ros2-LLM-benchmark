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
#include <chrono>

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
#include "nav_msgs/msg/occupancy_grid.hpp"
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
using namespace std::chrono_literals;

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

    // Pose-generating function used to uniformly distribute particles over
    // the map
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

    //parameter for which odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    geometry_msgs::msg::PoseStamped latest_odom_pose_;

    //parameter for which base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    rclcpp::Duration gui_publish_period{0, 0};
    rclcpp::Time save_pose_last_time;
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

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    rclcpp::Duration cloud_pub_interval{0, 0};
    rclcpp::Time last_cloud_pub_time;

    rclcpp::Duration bag_scan_period_{0, 0};

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(geometry_msgs::msg::PoseStamped& pose,
                     double& x, double& y, double& yaw,
                     const rclcpp::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    rclcpp::Duration transform_tolerance_{0, 0};

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;
    rclcpp::Service<nav_msgs::srv::SetMap>::SharedPtr set_map_srv_;
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
  //beam skip related params
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

    rclcpp::Time last_laser_received_ts_;
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
  // Save latest pose as we're shutting down.
  if (amcl_node_ptr) {
    amcl_node_ptr->savePoseToServer();
  }
  rclcpp::shutdown();
}

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  amcl_node_ptr = std::make_shared<AmclNode>();

  if (argc == 1)
  {
    // run using ROS input
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
        diagnosic_updater_(this),
        save_pose_last_time(this->now()),
        last_cloud_pub_time(this->now()),
        last_laser_received_ts_(this->now())
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server
  use_map_topic_ = this->declare_parameter("use_map_topic", false);
  first_map_only_ = this->declare_parameter("first_map_only", false);

  double tmp = this->declare_parameter("gui_publish_rate", -1.0);
  gui_publish_period = rclcpp::Duration::from_seconds(1.0/tmp);
  tmp = this->declare_parameter("save_pose_rate", 0.5);
  save_pose_period = rclcpp::Duration::from_seconds(1.0/tmp);

  laser_min_range_ = this->declare_parameter("laser_min_range", -1.0);
  laser_max_range_ = this->declare_parameter("laser_max_range", -1.0);
  max_beams_ = this->declare_parameter("laser_max_beams", 30);
  min_particles_ = this->declare_parameter("min_particles", 100);
  max_particles_ = this->declare_parameter("max_particles", 5000);
  pf_err_ = this->declare_parameter("kld_err", 0.01);
  pf_z_ = this->declare_parameter("kld_z", 0.99);
  alpha1_ = this->declare_parameter("odom_alpha1", 0.2);
  alpha2_ = this->declare_parameter("odom_alpha2", 0.2);
  alpha3_ = this->declare_parameter("odom_alpha3", 0.2);
  alpha4_ = this->declare_parameter("odom_alpha4", 0.2);
  alpha5_ = this->declare_parameter("odom_alpha5", 0.2);
  
  do_beamskip_ = this->declare_parameter("do_beamskip", false);
  beam_skip_distance_ = this->declare_parameter("beam_skip_distance", 0.5);
  beam_skip_threshold_ = this->declare_parameter("beam_skip_threshold", 0.3);
  beam_skip_error_threshold_ = this->declare_parameter("beam_skip_error_threshold", 0.9);

  z_hit_ = this->declare_parameter("laser_z_hit", 0.95);
  z_short_ = this->declare_parameter("laser_z_short", 0.1);
  z_max_ = this->declare_parameter("laser_z_max", 0.05);
  z_rand_ = this->declare_parameter("laser_z_rand", 0.05);
  sigma_hit_ = this->declare_parameter("laser_sigma_hit", 0.2);
  lambda_short_ = this->declare_parameter("laser_lambda_short", 0.1);
  laser_likelihood_max_dist_ = this->declare_parameter("laser_likelihood_max_dist", 2.0);
  
  std::string tmp_model_type = this->declare_parameter("laser_model_type", std::string("likelihood_field"));
  if(tmp_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(tmp_model_type == "likelihood_field_prob"){
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown laser model type \"%s\"; defaulting to likelihood_field model",
             tmp_model_type.c_str());
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  tmp_model_type = this->declare_parameter("odom_model_type", std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  d_thresh_ = this->declare_parameter("update_min_d", 0.2);
  a_thresh_ = this->declare_parameter("update_min_a", M_PI/6.0);
  odom_frame_id_ = this->declare_parameter("odom_frame_id", std::string("odom"));
  base_frame_id_ = this->declare_parameter("base_frame_id", std::string("base_link"));
  global_frame_id_ = this->declare_parameter("global_frame_id", std::string("map"));
  resample_interval_ = this->declare_parameter("resample_interval", 2);
  selective_resampling_ = this->declare_parameter("selective_resampling", false);
  
  double tmp_tol = this->declare_parameter("transform_tolerance", 0.1);
  alpha_slow_ = this->declare_parameter("recovery_alpha_slow", 0.001);
  alpha_fast_ = this->declare_parameter("recovery_alpha_fast", 0.1);
  tf_broadcast_ = this->declare_parameter("tf_broadcast", true);
  force_update_after_initialpose_ = this->declare_parameter("force_update_after_initialpose", false);
  force_update_after_set_map_ = this->declare_parameter("force_update_after_set_map", false);

  // For diagnostics
  std_warn_level_x_ = this->declare_parameter("std_warn_level_x", 0.2);
  std_warn_level_y_ = this->declare_parameter("std_warn_level_y", 0.2);
  std_warn_level_yaw_ = this->declare_parameter("std_warn_level_yaw", 0.1);

  transform_tolerance_ = rclcpp::Duration::from_seconds(tmp_tol);

  double bag_scan_period = this->declare_parameter("bag_scan_period", -1.0);
  bag_scan_period_ = rclcpp::Duration::from_seconds(bag_scan_period);

  odom_frame_id_ = stripSlash(odom_frame_id_);
  base_frame_id_ = stripSlash(base_frame_id_);
  global_frame_id_ = stripSlash(global_frame_id_);

  updatePoseFromServer();

  cloud_pub_interval = rclcpp::Duration::from_seconds(1.0);
  tfb_.reset(new tf2_ros::TransformBroadcaster(this));
  tf_.reset(new tf2_ros::Buffer(this->get_clock()));
  tfl_.reset(new tf2_ros::TransformListener(*tf_));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 2);
  
  global_loc_srv_ = this->create_service<std_srvs::srv::Empty>("global_localization", 
					 std::bind(&AmclNode::globalLocalizationCallback, this, std::placeholders::_1, std::placeholders::_2));
  nomotion_update_srv_= this->create_service<std_srvs::srv::Empty>("request_nomotion_update", 
           std::bind(&AmclNode::nomotionUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_map_srv_= this->create_service<nav_msgs::srv::SetMap>("set_map", 
           std::bind(&AmclNode::setMapCallback, this, std::placeholders::_1, std::placeholders::_2));

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(this, scan_topic_, rmw_qos_profile_default);
  laser_scan_filter_ = 
          new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(*laser_scan_sub_,
                                                             *tf_,
                                                             odom_frame_id_,
                                                             100,
                                                             this->get_node_logging_interface(),
                                                             this->get_node_clock_interface());
  laser_scan_filter_->registerCallback(std::bind(&AmclNode::laserReceived,
                                                 this, std::placeholders::_1));
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 2, std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

  if(use_map_topic_) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to map topic.");
  } else {
    requestMap();
  }
  m_force_update = false;

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = rclcpp::Duration::from_seconds(15.0);
  check_laser_timer_ = this->create_wall_timer(std::chrono::seconds(15), 
                                       std::bind(&AmclNode::checkLaserReceived, this));

  diagnosic_updater_.setHardwareID("None");
  diagnosic_updater_.add("Standard deviation", this, &AmclNode::standardDeviationDiagnostics);
}

void AmclNode::runFromBag(const std::string &in_bag_fn, bool trigger_global_localization)
{
  RCLCPP_WARN(this->get_logger(), "runFromBag is not fully implemented in this ROS2 migration.");
}

void AmclNode::savePoseToServer()
{
  tf2::Transform odom_pose_tf2;
  tf2::convert(latest_odom_pose_.pose, odom_pose_tf2);
  tf2::Transform map_pose = latest_tf_.inverse() * odom_pose_tf2;

  double yaw = tf2::getYaw(map_pose.getRotation());

  RCLCPP_DEBUG(this->get_logger(), "Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );

  // In ROS2, parameters are typically saved via a parameter service or file, 
  // but we can set them locally.
  this->set_parameter(rclcpp::Parameter("initial_pose_x", map_pose.getOrigin().x()));
  this->set_parameter(rclcpp::Parameter("initial_pose_y", map_pose.getOrigin().y()));
  this->set_parameter(rclcpp::Parameter("initial_pose_a", yaw));
  this->set_parameter(rclcpp::Parameter("initial_cov_xx", last_published_pose.pose.covariance[6*0+0]));
  this->set_parameter(rclcpp::Parameter("initial_cov_yy", last_published_pose.pose.covariance[6*1+1]));
  this->set_parameter(rclcpp::Parameter("initial_cov_aa", last_published_pose.pose.covariance[6*5+5]));
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = this->declare_parameter("initial_pose_x", 0.0);
  init_pose_[1] = this->declare_parameter("initial_pose_y", 0.0);
  init_pose_[2] = this->declare_parameter("initial_pose_a", 0.0);
  init_cov_[0] = this->declare_parameter("initial_cov_xx", 0.5 * 0.5);
  init_cov_[1] = this->declare_parameter("initial_cov_yy", 0.5 * 0.5);
  init_cov_[2] = this->declare_parameter("initial_cov_aa", (M_PI/12.0) * (M_PI/12.0));
}

void 
AmclNode::checkLaserReceived()
{
  rclcpp::Duration d = this->now() - last_laser_received_ts_;
  if(d > laser_check_interval_)
  {
    RCLCPP_WARN(this->get_logger(), "No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.seconds(),
             scan_topic_.c_str());
  }
}

void
AmclNode::requestMap()
{
  auto client = this->create_client<nav_msgs::srv::GetMap>("static_map");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    handleMapMessage(result->map);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service static_map");
  }
}

void
AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  RCLCPP_INFO(this->get_logger(), "Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);
  
  if(msg.header.frame_id != global_frame_id_)
    RCLCPP_WARN(this->get_logger(), "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could cause issues with reading published topics",
             msg.header.frame_id.c_str(),
             global_frame_id_.c_str());

  freeMapDependentMemory();
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
#endif
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_set_selective_resampling(pf_, selective_resampling_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  updatePoseFromServer();
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  delete odom_;
  odom_ = new AMCLOdom();
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    RCLCPP_INFO(this->get_logger(), "Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
					laser_likelihood_max_dist_, 
					do_beamskip_, beam_skip_distance_, 
					beam_skip_threshold_, beam_skip_error_threshold_);
    RCLCPP_INFO(this->get_logger(), "Done initializing likelihood field model.");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    RCLCPP_INFO(this->get_logger(), "Done initializing likelihood field model.");
  }

  applyInitialPose();
}

void
AmclNode::freeMapDependentMemory()
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

map_t*
AmclNode::convertMap( const nav_msgs::msg::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

AmclNode::~AmclNode()
{
  freeMapDependentMemory();
  delete laser_scan_filter_;
  delete laser_scan_sub_;
}

bool
AmclNode::getOdomPose(geometry_msgs::msg::PoseStamped& odom_pose,
                      double& x, double& y, double& yaw,
                      const rclcpp::Time& t, const std::string& f)
{
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = stripSlash(f);
  ident.header.stamp = t;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try
  {
    this->tf_->transform(ident, odom_pose, odom_frame_id_);
  }
  catch(const tf2::TransformException& e)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}


pf_vector_t
AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}

void
AmclNode::globalLocalizationCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  if( map_ == NULL ) {
    return;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  RCLCPP_INFO(this->get_logger(), "Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)map_);
  RCLCPP_INFO(this->get_logger(), "Global initialisation done!");
  pf_init_ = false;
}

void 
AmclNode::nomotionUpdateCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
	m_force_update = true;
}

void
AmclNode::setMapCallback(const std::shared_ptr<nav_msgs::srv::SetMap::Request> req,
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

void
AmclNode::laserReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan)
{
  std::string laser_scan_frame_id = stripSlash(laser_scan->header.frame_id);
  last_laser_received_ts_ = this->now();
  if( map_ == NULL ) {
    return;
  }
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  if(frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end())
  {
    RCLCPP_DEBUG(this->get_logger(), "Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan_frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    geometry_msgs::msg::PoseStamped ident;
    ident.header.frame_id = laser_scan_frame_id;
    ident.header.stamp = rclcpp::Time(0);
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

    geometry_msgs::msg::PoseStamped laser_pose;
    try
    {
      this->tf_->transform(ident, laser_pose, base_frame_id_);
    }
    catch(const tf2::TransformException& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan_frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.pose.position.x;
    laser_pose_v.v[1] = laser_pose.pose.position.y;
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    RCLCPP_DEBUG(this->get_logger(), "Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan_frame_id] = laser_index;
  } else {
    laser_index = frame_to_laser_[laser_scan_frame_id];
  }

  pf_vector_t pose;
  if(!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(this->get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();

  if(pf_init_)
  {
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    bool update = fabs(delta.v[0]) > d_thresh_ ||
                  fabs(delta.v[1]) > d_thresh_ ||
                  fabs(delta.v[2]) > a_thresh_;
    update = update || m_force_update;
    m_force_update=false;

    if(update)
      for(unsigned int i=0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    pf_odom_pose_ = pose;
    pf_init_ = true;

    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;

    force_publication = true;
    resample_count_ = 0;
  }
  else if(pf_init_ && lasers_update_[laser_index])
  {
    AMCLOdomData odata;
    odata.pose = pose;
    odata.delta = delta;
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);
  }

  bool resampled = false;
  if(lasers_update_[laser_index])
  {
    AMCLLaserData ldata;
    ldata.sensor = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan->angle_min);
    geometry_msgs::msg::QuaternionStamped min_q, inc_q;
    min_q.header.stamp = laser_scan->header.stamp;
    min_q.header.frame_id = stripSlash(laser_scan->header.frame_id);
    tf2::convert(q, min_q.quaternion);

    q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
    inc_q.header = min_q.header;
    tf2::convert(q, inc_q.quaternion);
    try
    {
      tf_->transform(min_q, min_q, base_frame_id_);
      tf_->transform(inc_q, inc_q, base_frame_id_);
    }
    catch(const tf2::TransformException& e)
    {
      RCLCPP_WARN(this->get_logger(), "Unable to transform min/max laser angles into base frame: %s",
               e.what());
      return;
    }

    double angle_min = tf2::getYaw(min_q.quaternion);
    double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

    angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

    RCLCPP_DEBUG(this->get_logger(), "Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

    if(laser_max_range_ > 0.0)
      ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
    else
      ldata.range_max = laser_scan->range_max;
    double range_min;
    if(laser_min_range_ > 0.0)
      range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
    else
      range_min = laser_scan->range_min;

    if(ldata.range_max <= 0.0 || range_min < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "range_max or range_min from laser is negative! ignore this message.");
      return;
    }

    ldata.ranges = new double[ldata.range_count][2];
    for(int i=0;i<ldata.range_count;i++)
    {
      if(laser_scan->ranges[i] <= range_min)
        ldata.ranges[i][0] = ldata.range_max;
      else if(laser_scan->ranges[i] > ldata.range_max)
        ldata.ranges[i][0] = std::numeric_limits<decltype(ldata.range_max)>::max();
      else
        ldata.ranges[i][0] = laser_scan->ranges[i];
      ldata.ranges[i][1] = angle_min + (i * angle_increment);
    }

    lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
    lasers_update_[laser_index] = false;
    pf_odom_pose_ = pose;

    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    RCLCPP_DEBUG(this->get_logger(), "Num samples: %d\n", set->sample_count);

    if (!m_force_update)
    {
      geometry_msgs::msg::PoseArray cloud_msg;
      cloud_msg.header.stamp = this->now();
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.poses.resize(set->sample_count);
      for(int i=0;i<set->sample_count;i++)
      {
        cloud_msg.poses[i].position.x = set->samples[i].pose.v[0];
        cloud_msg.poses[i].position.y = set->samples[i].pose.v[1];
        cloud_msg.poses[i].position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, set->samples[i].pose.v[2]);
        tf2::convert(q, cloud_msg.poses[i].orientation);
      }
      particlecloud_pub_->publish(cloud_msg);
    }
  }

  if(resampled || force_publication)
  {
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0;
        hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        RCLCPP_ERROR(this->get_logger(), "Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      RCLCPP_DEBUG(this->get_logger(), "Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      geometry_msgs::msg::PoseWithCovarianceStamped p;
      p.header.frame_id = global_frame_id_;
      p.header.stamp = laser_scan->header.stamp;
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];

      tf2::Quaternion q;
      q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
      tf2::convert(q, p.pose.pose.orientation);
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      p.pose.covariance[6*5+5] = set->cov.m[2][2];

      pose_pub_->publish(p);
      last_published_pose = p;

      RCLCPP_DEBUG(this->get_logger(), "New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);

      geometry_msgs::msg::PoseStamped odom_to_map;
      try
      {
        tf2::Quaternion q;
        q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
        tf2::Transform tmp_tf(q, tf2::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                              hyps[max_weight_hyp].pf_pose_mean.v[1],
                                              0.0));

        geometry_msgs::msg::PoseStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = base_frame_id_;
        tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
        tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

        this->tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
      }
      catch(const tf2::TransformException&)
      {
        RCLCPP_DEBUG(this->get_logger(), "Failed to subtract base to odom transform");
        return;
      }

      tf2::convert(odom_to_map.pose, latest_tf_);
      latest_tf_valid_ = true;

      if (tf_broadcast_ == true)
      {
        rclcpp::Time transform_expiration = (rclcpp::Time(laser_scan->header.stamp) +
                                          transform_tolerance_);
        geometry_msgs::msg::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = global_frame_id_;
        tmp_tf_stamped.header.stamp = transform_expiration;
        tmp_tf_stamped.child_frame_id = odom_frame_id_;
        tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

        this->tfb_->sendTransform(tmp_tf_stamped);
        sent_first_transform_ = true;
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "No pose!");
    }
  }
  else if(latest_tf_valid_)
  {
    if (tf_broadcast_ == true)
    {
      rclcpp::Time transform_expiration = (rclcpp::Time(laser_scan->header.stamp) +
                                        transform_tolerance_);
      geometry_msgs::msg::TransformStamped tmp_tf_stamped;
      tmp_tf_stamped.header.frame_id = global_frame_id_;
      tmp_tf_stamped.header.stamp = transform_expiration;
      tmp_tf_stamped.child_frame_id = odom_frame_id_;
      tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
      this->tfb_->sendTransform(tmp_tf_stamped);
    }

    rclcpp::Time now = this->now();
    if((save_pose_period.seconds() > 0.0) &&
       (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }

  diagnosic_updater_.force_update();
}

void
AmclNode::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{
  handleInitialPoseMessage(*msg);
  if (force_update_after_initialpose_)
  {
    m_force_update = true;
  }
}

void
AmclNode::handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if(msg.header.frame_id == "")
  {
    RCLCPP_WARN(this->get_logger(), "Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  else if(stripSlash(msg.header.frame_id) != global_frame_id_)
  {
    RCLCPP_WARN(this->get_logger(), "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             stripSlash(msg.header.frame_id).c_str(),
             global_frame_id_.c_str());
    return;
  }

  geometry_msgs::msg::TransformStamped tx_odom;
  try
  {
    tx_odom = tf_->lookupTransform(base_frame_id_, msg.header.stamp,
                                   base_frame_id_, this->now(),
                                   odom_frame_id_, rclcpp::Duration::from_seconds(0.5));
  }
  catch(const tf2::TransformException& e)
  {
    if(sent_first_transform_)
      RCLCPP_WARN(this->get_logger(), "Failed to transform initial pose in time (%s)", e.what());
    tf2::convert(tf2::Transform::getIdentity(), tx_odom.transform);
  }

  tf2::Transform tx_odom_tf2;
  tf2::convert(tx_odom.transform, tx_odom_tf2);
  tf2::Transform pose_old, pose_new;
  tf2::convert(msg.pose.pose, pose_old);
  pose_new = pose_old * tx_odom_tf2;

  RCLCPP_INFO(this->get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f",
           this->now().seconds(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           tf2::getYaw(pose_new.getRotation()));
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  applyInitialPose();
}

void
AmclNode::applyInitialPose()
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if( initial_pose_hyp_ != NULL && map_ != NULL ) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;

    delete initial_pose_hyp_;
    initial_pose_hyp_ = NULL;
  }
}

void
AmclNode::standardDeviationDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
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