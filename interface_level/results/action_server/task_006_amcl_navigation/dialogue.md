# Task: action_server/task_006_amcl_navigation

```cpp
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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure is not available in ROS2, so this part is omitted

// Allows AMCL to run from bag file
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rcutils/time.h>

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

    /**
     * @brief Uses TF and LaserScan messages from bag file to drive AMCL instead
     * @param in_bag_fn input bagfile
     * @param trigger_global_localization whether to trigger global localization
     * before starting to process the bagfile
     */
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
    bool globalLocalizationCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool nomotionUpdateCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool setMapCallback(const std::shared_ptr<nav_msgs::srv::SetMap::Request> req,
                        std::shared_ptr<nav_msgs::srv::SetMap::Response> res);

    void laserReceived(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
    void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
    void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

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

    rclcpp::Duration gui_publish_period;
    rclcpp::Time save_pose_last_time;
    rclcpp::Duration save_pose_period;

    geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
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

    rclcpp::Duration cloud_pub_interval;
    rclcpp::Time last_cloud_pub_time;

    // For slowing play-back when reading directly from a bag file
    rclcpp::Duration bag_scan_period_;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(geometry_msgs::msg::PoseStamped& pose,
                     double& x, double& y, double& yaw,
                     const rclcpp::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    rclcpp::Duration transform_tolerance_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_; //to let amcl update samples without requiring motion
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

    void reconfigureCB(); // Dynamic reconfigure not supported in ROS2, so this is a placeholder

    rclcpp::Time last_laser_received_ts_;
    rclcpp::Duration laser_check_interval_;
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
  if (amcl_node_ptr)
    amcl_node_ptr->savePoseToServer();
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

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();

  // To quote Morgan, Hooray!
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
        first_reconfigure_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server
  this->declare_parameter("use_map_topic", false);
  this->declare_parameter("first_map_only", false);
  this->declare_parameter("gui_publish_rate", -1.0);
  this->declare_parameter("save_pose_rate", 0.5);
  this->declare_parameter("laser_min_range", -1.0);
  this->declare_parameter("laser_max_range", -1.0);
  this->declare_parameter("laser_max_beams", 30);
  this->declare_parameter("min_particles", 100);
  this->declare_parameter("max_particles", 5000);
  this->declare_parameter("kld_err", 0.01);
  this->declare_parameter("kld_z", 0.99);
  this->declare_parameter("odom_alpha1", 0.2);
  this->declare_parameter("odom_alpha2", 0.2);
  this->declare_parameter("odom_alpha3", 0.2);
  this->declare_parameter("odom_alpha4", 0.2);
  this->declare_parameter("odom_alpha5", 0.2);
  this->declare_parameter("do_beamskip", false);
  this->declare_parameter("beam_skip_distance", 0.5);
  this->declare_parameter("beam_skip_threshold", 0.3);
  this->declare_parameter("beam_skip_error_threshold", 0.9);
  this->declare_parameter("laser_z_hit", 0.95);
  this->declare_parameter("laser_z_short", 0.1);
  this->declare_parameter("laser_z_max", 0.05);
  this->declare_parameter("laser_z_rand", 0.05);
  this->declare_parameter("laser_sigma_hit", 0.2);
  this->declare_parameter("laser_lambda_short", 0.1);
  this->declare_parameter("laser_likelihood_max_dist", 2.0);
  this->declare_parameter("laser_model_type", "likelihood_field");
  this->declare_parameter("odom_model_type", "diff");
  this->declare_parameter("update_min_d", 0.2);
  this->declare_parameter("update_min_a", M_PI/6.0);
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("base_frame_id", "base_link");
  this->declare_parameter("global_frame_id", "map");
  this->declare_parameter("resample_interval", 2);
  this->declare_parameter("selective_resampling", false);
  this->declare_parameter("transform_tolerance", 0.1);
  this->declare_parameter("recovery_alpha_slow", 0.001);
  this->declare_parameter("recovery_alpha_fast", 0.1);
  this->declare_parameter("tf_broadcast", true);
  this->declare_parameter("force_update_after_initialpose", false);
  this->declare_parameter("force_update_after_set_map", false);
  this->declare_parameter("std_warn_level_x", 0.2);
  this->declare_parameter("std_warn_level_y", 0.2);
  this->declare_parameter("std_warn_level_yaw", 0.1);
  this->declare_parameter("bag_scan_period", -1.0);

  this->get_parameter("use_map_topic", use_map_topic_);
  this->get_parameter("first_map_only", first_map_only_);

  double tmp;
  this->get_parameter("gui_publish_rate", tmp);
  gui_publish_period = rclcpp::Duration::from_seconds(tmp > 0 ? 1.0/tmp : 0.0);
  this->get_parameter("save_pose_rate", tmp);
  save_pose_period = rclcpp::Duration::from_seconds(tmp > 0 ? 1.0/tmp : 0.0);

  this->get_parameter("laser_min_range", laser_min_range_);
  this->get_parameter("laser_max_range", laser_max_range_);
  this->get_parameter("laser_max_beams", max_beams_);
  this->get_parameter("min_particles", min_particles_);
  this->get_parameter("max_particles", max_particles_);
  this->get_parameter("kld_err", pf_err_);
  this->get_parameter("kld_z", pf_z_);
  this->get_parameter("odom_alpha1", alpha1_);
  this->get_parameter("odom_alpha2", alpha2_);
  this->get_parameter("odom_alpha3", alpha3_);
  this->get_parameter("odom_alpha4", alpha4_);
  this->get_parameter("odom_alpha5", alpha5_);

  this->get_parameter("do_beamskip", do_beamskip_);
  this->get_parameter("beam_skip_distance", beam_skip_distance_);
  this->get_parameter("beam_skip_threshold", beam_skip_threshold_);
  this->get_parameter("beam_skip_error_threshold", beam_skip_error_threshold_);

  this->get_parameter("laser_z_hit", z_hit_);
  this->get_parameter("laser_z_short", z_short_);
  this->get_parameter("laser_z_max", z_max_);
  this->get_parameter("laser_z_rand", z_rand_);
  this->get_parameter("laser_sigma_hit", sigma_hit_);
  this->get_parameter("laser_lambda_short", lambda_short_);
  this->get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);

  std::string tmp_model_type;
  this->get_parameter("laser_model_type", tmp_model_type);
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

  this->get_parameter("odom_model_type", tmp_model_type);
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

  this->get_parameter("update_min_d", d_thresh_);
  this->get_parameter("update_min_a", a_thresh_);
  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("base_frame_id", base_frame_id_);
  this->get_parameter("global_frame_id", global_frame_id_);
  this->get_parameter("resample_interval", resample_interval_);
  this->get_parameter("selective_resampling", selective_resampling_);
  double tmp_tol;
  this->get_parameter("transform_tolerance", tmp_tol);
  this->get_parameter("recovery_alpha_slow", alpha_slow_);
  this->get_parameter("recovery_alpha_fast", alpha_fast_);
  this->get_parameter("tf_broadcast", tf_broadcast_);
  this->get_parameter("force_update_after_initialpose", force_update_after_initialpose_);
  this->get_parameter("force_update_after_set_map", force_update_after_set_map_);

  this->get_parameter("std_warn_level_x", std_warn_level_x_);
  this->get_parameter("std_warn_level_y", std_warn_level_y_);
  this->get_parameter("std_warn_level_yaw", std_warn_level_yaw_);

  transform_tolerance_ = rclcpp::Duration::from_seconds(tmp_tol);

  {
    double bag_scan_period;
    this->get_parameter("bag_scan_period", bag_scan_period);
    bag_scan_period_ = rclcpp::Duration::from_seconds(bag_scan_period);
  }

  odom_frame_id_ = stripSlash(odom_frame_id_);
  base_frame_id_ = stripSlash(base_frame_id_);
  global_frame_id_ = stripSlash(global_frame_id_);

  updatePoseFromServer();

  cloud_pub_interval = rclcpp::Duration::from_seconds(1.0);
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", rclcpp::QoS(2).transient_local());
  particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", rclcpp::QoS(2).transient_local());
  global_loc_srv_ = this->create_service<std_srvs::srv::Empty>("global_localization", std::bind(&AmclNode::globalLocalizationCallback, this, std::placeholders::_1, std::placeholders::_2));
  nomotion_update_srv_= this->create_service<std_srvs::srv::Empty>("request_nomotion_update", std::bind(&AmclNode::nomotionUpdateCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_map_srv_= this->create_service<nav_msgs::srv::SetMap>("set_map", std::bind(&AmclNode::setMapCallback, this, std::placeholders::_1, std::placeholders::_2));

  laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, scan_topic_, rmw_qos_profile_sensor_data);
  laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(*laser_scan_sub_, *tf_, odom_frame_id_, 100, this);
  laser_scan_filter_->registerCallback(std::bind(&AmclNode::laserReceived, this, std::placeholders::_1));
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::QoS(2), std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

  if(use_map_topic_) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1), std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to map topic.");
  } else {
    requestMap();
  }
  m_force_update = false;

  // Dynamic reconfigure is not supported in ROS2, so reconfigureCB is not used

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = rclcpp::Duration::from_seconds(15.0);
  check_laser_timer_ = this->create_wall_timer(laser_check_interval_, std::bind(&AmclNode::checkLaserReceived, this));

  diagnosic_updater_.setHardwareID("None");
  diagnosic_updater_.add("Standard deviation", this, &AmclNode::standardDeviationDiagnostics);
}

void AmclNode::reconfigureCB()
{
  // Dynamic reconfigure is not supported in ROS2, so this is a placeholder
}

void AmclNode::runFromBag(const std::string &in_bag_fn, bool trigger_global_localization)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(this->shared_from_this());

  rosbag2_cpp::Reader bag_reader;
  bag_reader.open(in_bag_fn);

  std::vector<std::string> topics = {"tf", "base_scan"};

  // Publishers for replaying bag messages
  auto laser_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("base_scan", rclcpp::QoS(100));
  auto tf_pub = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));

  // Sleep for a second to let all subscribers connect
  rclcpp::sleep_for(std::chrono::seconds(1));

  rclcpp::Time start = this->now();

  // Wait for map
  while (rclcpp::ok())
  {
    {
      boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
      if (map_)
      {
        RCLCPP_INFO(this->get_logger(), "Map is ready");
        break;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for the map...");
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (trigger_global_localization)
  {
    auto empty_srv = std::make_shared<std_srvs::srv::Empty::Request>();
    auto empty_res = std::make_shared<std_srvs::srv::Empty::Response>();
    globalLocalizationCallback(empty_srv, empty_res);
  }

  while (bag_reader.has_next())
  {
    if (!rclcpp::ok())
    {
      break;
    }

    auto bag_message = bag_reader.read_next();

    // Process any ros messages or callbacks at this point
    rclcpp::spin_some(this->get_node_base_interface());

    if (bag_message->topic == "/tf" || bag_message->topic == "tf")
    {
      auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
      serializer.deserialize_message(&serialized_msg, tf_msg.get());

      tf_pub->publish(*tf_msg);
      for (size_t ii=0; ii<tf_msg->transforms.size(); ++ii)
      {
        tf_->setTransform(tf_msg->transforms[ii], "rosbag_authority");
      }
      continue;
    }

    if (bag_message->topic == "base_scan")
    {
      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;
      serializer.deserialize_message(&serialized_msg, scan_msg.get());

      laser_pub->publish(*scan_msg);
      laser_scan_filter_->add(scan_msg);
      if (bag_scan_period_ > rclcpp::Duration(0))
      {
        rclcpp::sleep_for(std::chrono::nanoseconds(bag_scan_period_.nanoseconds()));
      }
      continue;
    }

    RCLCPP_WARN(this->get_logger(), "Unsupported message type %s", bag_message->topic.c_str());
  }

  double runtime = (this->now() - start).seconds();
  RCLCPP_INFO(this->get_logger(), "Bag complete, took %.1f seconds to process, shutting down", runtime);

  const geometry_msgs::msg::Quaternion & q = last_published_pose.pose.pose.orientation;
  double yaw, pitch, roll;
  tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getEulerYPR(yaw,pitch,roll);
  RCLCPP_INFO(this->get_logger(), "Final location %.3f, %.3f, %.3f with stamp=%f",
            last_published_pose.pose.pose.position.x,
            last_published_pose.pose.pose.position.y,
            yaw, last_published_pose.header.stamp.seconds()
            );

  rclcpp::shutdown();
}


void AmclNode::savePoseToServer()
{
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf2::Transform odom_pose_tf2;
  tf2::fromMsg(latest_odom_pose_.pose, odom_pose_tf2);
  tf2::Transform map_pose = latest_tf_.inverse() * odom_pose_tf2;

  double yaw = tf2::getYaw(map_pose.getRotation());

  RCLCPP_DEBUG(this->get_logger(), "Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );

  this->set_parameter(rclcpp::Parameter("initial_pose_x", map_pose.getOrigin().x()));
  this->set_parameter(rclcpp::Parameter("initial_pose_y", map_pose.getOrigin().y()));
  this->set_parameter(rclcpp::Parameter("initial_pose_a", yaw));
  this->set_parameter(rclcpp::Parameter("initial_cov_xx", 
                                  last_published_pose.pose.covariance[6*0+0]));
  this->set_parameter(rclcpp::Parameter("initial_cov_yy", 
                                  last_published_pose.pose.covariance[6*1+1]));
  this->set_parameter(rclcpp::Parameter("initial_cov_aa", 
                                  last_published_pose.pose.covariance[6*5+5]));
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  // Check for NAN on input from param server, #5239
  double tmp_pos;
  if (this->get_parameter("initial_pose_x", tmp_pos))
  {
    if(!std::isnan(tmp_pos))
      init_pose_[0] = tmp_pos;
    else 
      RCLCPP_WARN(this->get_logger(), "ignoring NAN in initial pose X position");
  }
  if (this->get_parameter("initial_pose_y", tmp_pos))
  {
    if(!std::isnan(tmp_pos))
      init_pose_[1] = tmp_pos;
    else
      RCLCPP_WARN(this->get_logger(), "ignoring NAN in initial pose Y position");
  }
  if (this->get_parameter("initial_pose_a", tmp_pos))
  {
    if(!std::isnan(tmp_pos))
      init_pose_[2] = tmp_pos;
    else
      RCLCPP_WARN(this->get_logger(), "ignoring NAN in initial pose Yaw");
  }
  if (this->get_parameter("initial_cov_xx", tmp_pos))
  {
    if(!std::isnan(tmp_pos))
      init_cov_[0] =tmp_pos;
    else
      RCLCPP_WARN(this->get_logger(), "ignoring NAN in initial covariance XX");
  }
  if (this->get_parameter("initial_cov_yy", tmp_pos))
  {
    if(!std::isnan(tmp_pos))
      init_cov_[1] = tmp_pos;
    else
      RCLCPP_WARN(this->get_logger(), "ignoring NAN in initial covariance YY");
  }
  if (this->get_parameter("initial_cov_aa", tmp_pos))
  {
    if(!std::isnan(tmp_pos))
      init_cov_[2] = tmp_pos;
    else
      RCLCPP_WARN(this->get_logger(), "ignoring NAN in initial covariance AA");	
  }
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
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  auto client = this->create_client<nav_msgs::srv::GetMap>("static_map");
  RCLCPP_INFO(this->get_logger(), "Requesting the map...");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Request for map failed; trying again...");
  }
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    handleMapMessage(result.get()->map);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service static_map");
  }
}

void
AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
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
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
#endif
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_set_selective_resampling(pf_, selective_resampling_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
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

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  RCLCPP_ASSERT(this->get_logger(), odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  RCLCPP_ASSERT(this->get_logger(), laser_);
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

  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
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

/**
 * Convert an OccupancyGrid map message into the internal
 * representation. This allocates a map_t and returns it.
 */
map_t*
AmclNode::convertMap( const nav_msgs::msg::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  RCLCPP_ASSERT(this->get_logger(), map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  RCLCPP_ASSERT(this->get_logger(), map->cells);
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
  // unique_ptr will clean laser_scan_filter_ and laser_scan_sub_
}

bool
AmclNode::getOdomPose(geometry_msgs::msg::PoseStamped& odom_pose,
                      double& x, double& y, double& yaw,
                      const rclcpp::Time& t, const std::string& f)
{
  // Get the robot's pose
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

  RCLCPP_DEBUG(rclcpp::get_logger("amcl"), "Generating new uniform sample");
  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}

bool
AmclNode::globalLocalizationCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  if( map_ == NULL ) {
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  RCLCPP_INFO(this->get_logger(), "Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)map_);
  RCLCPP_INFO(this->get_logger(), "Global initialisation done!");
  pf_init_ = false;
  return true;
}

// force nomotion updates (amcl updating without requiring motion)
bool 
AmclNode::nomotionUpdateCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
	m_force_update = true;
	//RCLCPP_INFO(this->get_logger(), "Requesting no-motion update");
	return true;
}

bool
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
  return true;
}

void
AmclNode::laserReceived(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  std::string laser_scan_frame_id = stripSlash(laser_scan->header.frame_id);
  last_laser_received_ts_ = this->now();
  if( map_ == NULL ) {
    return;
  }
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
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
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    RCLCPP_DEBUG(this->get_logger(), "Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan_frame_id] = laser_index;
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan_frame_id];
  }

  // Where was the robot when this scan was taken?
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
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > d_thresh_ ||
                  fabs(delta.v[1]) > d_thresh_ ||
                  fabs(delta.v[2]) > a_thresh_;
    update = update || m_force_update;
    m_force_update=false;

    // Set the laser update flags
    if(update)
      for(unsigned int i=0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;

    force_publication = true;

    resample_count_ = 0;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && lasers_update_[laser_index])
  {
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(lasers_update_[laser_index])
  {
  //TODO:Implement particle filter update for incoming laser scan
  // 1. Fill in laser sensor data structure (ranges, bearings)
  // 2. Call UpdateSensor on the AMCLLaser object
  // 3. Update odometry pose for filter
  // 4. Perform resampling if needed
  // 5. Publish updated particle cloud and estimated pose
  // END OF TODO

    // 1. Fill in laser sensor data structure (ranges, bearings)
    AMCLLaserData ldata;
    ldata.sensor = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();
    ldata.ranges = laser_scan->ranges.data();
    ldata.bearings = new double[ldata.range_count];
    for (size_t i = 0; i < ldata.range_count; i++) {
      ldata.bearings[i] = laser_scan->angle_min + i * laser_scan->angle_increment;
    }
    ldata.sensor_pose = lasers_[laser_index]->GetLaserPose();

    // 2. Call UpdateSensor on the AMCLLaser object
    lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata);

    // 3. Update odometry pose for filter
    pf_odom_pose_ = pose;

    // 4. Perform resampling if needed
    resample_count_++;
    if (resample_count_ % resample_interval_ == 0) {
      pf_update_resample(pf_);
      resampled = true;
    }

    // 5. Publish updated particle cloud and estimated pose
    // Publish particle cloud
    if (particlecloud_pub_->get_subscription_count() > 0) {
      geometry_msgs::msg::PoseArray cloud_msg;
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.header.stamp = laser_scan->header.stamp;
      cloud_msg.poses.resize(pf_->sets[pf_->current_set].sample_count);
      for (int i = 0; i < pf_->sets[pf_->current_set].sample_count; i++) {
        pf_sample_t* sample = pf_->sets[pf_->current_set].samples + i;
        geometry_msgs::msg::Pose pose;
        pose.position.x = sample->pose.v[0];
        pose.position.y = sample->pose.v[1];
        tf2::Quaternion q;
        q.setRPY(0, 0, sample->pose.v[2]);
        pose.orientation = tf2::toMsg(q);
        cloud_msg.poses[i] = pose;
      }
      particlecloud_pub_->publish(cloud_msg);
    }

    delete[] ldata.bearings;
  }

  if(resampled || force_publication)
  {
    // Read out the current hypotheses
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
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = laser_scan->header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];

      tf2::Quaternion q;
      q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
      p.pose.pose.orientation = tf2::toMsg(q);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6*5+5] = set->cov.m[2][2];

      pose_pub_->publish(p);
      last_published_pose = p;

      RCLCPP_DEBUG(this->get_logger(), "New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // subtracting base to odom from map to base and send map to odom instead
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
        tmp_tf_stamped.pose = tf2::toMsg(tmp_tf.inverse());

        this->tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
      }
      catch(const tf2::TransformException&)
      {
        RCLCPP_DEBUG(this->get_logger(), "Failed to subtract base to odom transform");
        return;
      }

      tf2::fromMsg(odom_to_map.pose, latest_tf_);
      latest_tf_valid_ = true;

      if (tf_broadcast_ == true)
      {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        rclcpp::Time transform_expiration = laser_scan->header.stamp + transform_tolerance_;
        geometry_msgs::msg::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = global_frame_id_;
        tmp_tf_stamped.header.stamp = transform_expiration;
        tmp_tf_stamped.child_frame_id = odom_frame_id_;
        tmp_tf_stamped.transform = tf2::toMsg(latest_tf_.inverse());

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
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      rclcpp::Time transform_expiration = laser_scan->header.stamp + transform_tolerance_;
      geometry_msgs::msg::TransformStamped tmp_tf_stamped;
      tmp_tf_stamped.header.frame_id = global_frame_id_;
      tmp_tf_stamped.header.stamp = transform_expiration;
      tmp_tf_stamped.child_frame_id = odom_frame_id_;
      tmp_tf_stamped.transform = tf2::toMsg(latest_tf_.inverse());
      this->tfb_->sendTransform(tmp_tf_stamped);
    }

    // Is it time to save our last pose to the param server
    rclcpp::Time now = this->now();
    if((save_pose_period.seconds() > 0.0) &&
       (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }

  diagnosic_updater_.update();
}

void
AmclNode::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
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
    // This should be removed at some point
    RCLCPP_WARN(this->get_logger(), "Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(stripSlash(msg.header.frame_id) != global_frame_id_)
  {
    RCLCPP_WARN(this->get_logger(), "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             stripSlash(msg.header.frame_id).c_str(),
             global_frame_id_.c_str());
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  geometry_msgs::msg::TransformStamped tx_odom;
  try
  {
    // wait a little for the latest tf to become available
    tx_odom = tf_->lookupTransform(base_frame_id_, msg.header.stamp,
                                   base_frame_id_, this->now(),
                                   odom_frame_id_, rclcpp::Duration::from_seconds(0.5));
  }
  catch(const tf2::TransformException& e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      RCLCPP_WARN(this->get_logger(), "Failed to transform initial pose in time (%s)", e.what());
    tx_odom.transform = tf2::toMsg(tf2::Transform::getIdentity());
  }

  tf2::Transform tx_odom_tf2;
  tf2::fromMsg(tx_odom.transform, tx_odom_tf2);
  tf2::Transform pose_old, pose_new;
  tf2::fromMsg(msg.pose.pose, pose_old);
  pose_new = pose_old * tx_odom_tf2;

  // Transform into the global frame

  RCLCPP_INFO(this->get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f",
           this->now().seconds(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           tf2::getYaw(pose_new.getRotation()));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
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

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
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
```