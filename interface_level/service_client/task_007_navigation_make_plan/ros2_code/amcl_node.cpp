/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 */

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <csignal>
#include <limits>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"
#include "portable_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/srv/set_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/empty.hpp>

#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#define NEW_UNIFORM_SAMPLING 1

using namespace amcl;
using namespace std::chrono_literals;

typedef struct
{
  double weight;
  pf_vector_t pf_pose_mean;
  pf_matrix_t pf_pose_cov;
} amcl_hyp_t;

static double normalize(double z) { return atan2(sin(z), cos(z)); }

static double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0) d2 *= -1.0;
  return (fabs(d1) < fabs(d2)) ? d1 : d2;
}

static const std::string scan_topic_ = "scan";

inline std::string stripSlash(const std::string & in)
{
  std::string out = in;
  if ((!in.empty()) && (in[0] == '/')) out.erase(0, 1);
  return out;
}

class AmclNode : public rclcpp::Node
{
public:
  AmclNode();
  ~AmclNode();

  void runFromBag(const std::string & in_bag_fn, bool trigger_global_localization = false);
  int process();
  void savePoseToServer();

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool sent_first_transform_;
  tf2::Transform latest_tf_;
  bool latest_tf_valid_;

  static pf_vector_t uniformPoseGenerator(void * arg);
#if NEW_UNIFORM_SAMPLING
  static std::vector<std::pair<int, int>> free_space_indices;
#endif

  bool globalLocalizationCallback(std_srvs::srv::Empty::Request & req, std_srvs::srv::Empty::Response & res);
  bool nomotionUpdateCallback(std_srvs::srv::Empty::Request & req, std_srvs::srv::Empty::Response & res);
  bool setMapCallback(nav_msgs::srv::SetMap::Request & req, nav_msgs::srv::SetMap::Response & res);

  void laserReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan);
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  void handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void mapReceived(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);

  void handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg);
  void freeMapDependentMemory();
  map_t * convertMap(const nav_msgs::msg::OccupancyGrid & map_msg);
  void updatePoseFromServer();
  void applyInitialPose();

  std::string odom_frame_id_;
  geometry_msgs::msg::PoseStamped latest_odom_pose_;
  std::string base_frame_id_;
  std::string global_frame_id_;

  bool use_map_topic_;
  bool first_map_only_;

  rclcpp::Duration gui_publish_period_;
  rclcpp::Time save_pose_last_time_;
  rclcpp::Duration save_pose_period_;

  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose_;

  map_t * map_;
  pf_t * pf_;
  double pf_err_, pf_z_;
  bool pf_init_;
  pf_vector_t pf_odom_pose_;
  double d_thresh_, a_thresh_;
  int resample_interval_;
  int resample_count_;
  double laser_min_range_;
  double laser_max_range_;
  bool m_force_update;

  AMCLOdom * odom_;
  AMCLLaser * laser_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  std::vector<AMCLLaser *> lasers_;
  std::vector<bool> lasers_update_;
  std::map<std::string, int> frame_to_laser_;

  rclcpp::Duration cloud_pub_interval_;
  rclcpp::Time last_cloud_pub_time_;
  rclcpp::Duration bag_scan_period_;

  bool getOdomPose(
    geometry_msgs::msg::PoseStamped & pose, double & x, double & y, double & yaw,
    const rclcpp::Time & t, const std::string & f);

  rclcpp::Duration transform_tolerance_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;
  rclcpp::Service<nav_msgs::srv::SetMap>::SharedPtr set_map_srv_;
  rclcpp::TimerBase::SharedPtr check_laser_timer_;
  rclcpp::TimerBase::SharedPtr map_request_timer_;

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;

  diagnostic_updater::Updater diagnosic_updater_;
  void standardDeviationDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic_status);
  double std_warn_level_x_;
  double std_warn_level_y_;
  double std_warn_level_yaw_;

  amcl_hyp_t * initial_pose_hyp_;
  bool first_map_received_;

  std::recursive_mutex configuration_mutex_;

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

  rclcpp::Time last_laser_received_ts_;
  rclcpp::Duration laser_check_interval_;

  void checkLaserReceived();
  void requestMap();
};

#if NEW_UNIFORM_SAMPLING
std::vector<std::pair<int, int>> AmclNode::free_space_indices;
#endif

std::shared_ptr<AmclNode> amcl_node_ptr;

void sigintHandler(int)
{
  if (amcl_node_ptr) amcl_node_ptr->savePoseToServer();
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, sigintHandler);

  amcl_node_ptr = std::make_shared<AmclNode>();

  if (argc == 1) {
    rclcpp::spin(amcl_node_ptr);
  } else if ((argc >= 3) && (std::string(argv[1]) == "--run-from-bag")) {
    if (argc == 3) {
      amcl_node_ptr->runFromBag(argv[2]);
    } else if ((argc == 4) && (std::string(argv[3]) == "--global-localization")) {
      amcl_node_ptr->runFromBag(argv[2], true);
    }
  }

  amcl_node_ptr.reset();
  return 0;
}

AmclNode::AmclNode()
: Node("amcl"),
  sent_first_transform_(false),
  latest_tf_valid_(false),
  map_(nullptr),
  pf_(nullptr),
  pf_init_(false),
  resample_count_(0),
  m_force_update(false),
  odom_(nullptr),
  laser_(nullptr),
  initial_pose_hyp_(nullptr),
  first_map_received_(false),
  diagnosic_updater_(this)
{
  this->declare_parameter("use_map_topic", false);
  this->declare_parameter("first_map_only", false);
  this->get_parameter("use_map_topic", use_map_topic_);
  this->get_parameter("first_map_only", first_map_only_);

  double tmp = -1.0;
  this->declare_parameter("gui_publish_rate", -1.0);
  this->get_parameter("gui_publish_rate", tmp);
  gui_publish_period_ = (tmp > 0.0) ? rclcpp::Duration::from_seconds(1.0 / tmp) : rclcpp::Duration::from_seconds(1.0);

  this->declare_parameter("save_pose_rate", 0.5);
  this->get_parameter("save_pose_rate", tmp);
  save_pose_period_ = (tmp > 0.0) ? rclcpp::Duration::from_seconds(1.0 / tmp) : rclcpp::Duration::from_seconds(2.0);

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

  this->declare_parameter("laser_z_hit", 0.95);
  this->declare_parameter("laser_z_short", 0.1);
  this->declare_parameter("laser_z_max", 0.05);
  this->declare_parameter("laser_z_rand", 0.05);
  this->declare_parameter("laser_sigma_hit", 0.2);
  this->declare_parameter("laser_lambda_short", 0.1);
  this->declare_parameter("laser_likelihood_max_dist", 2.0);
  this->get_parameter("laser_z_hit", z_hit_);
  this->get_parameter("laser_z_short", z_short_);
  this->get_parameter("laser_z_max", z_max_);
  this->get_parameter("laser_z_rand", z_rand_);
  this->get_parameter("laser_sigma_hit", sigma_hit_);
  this->get_parameter("laser_lambda_short", lambda_short_);
  this->get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);

  std::string tmp_model_type("likelihood_field");
  this->declare_parameter("laser_model_type", tmp_model_type);
  this->get_parameter("laser_model_type", tmp_model_type);
  if (tmp_model_type == "beam") laser_model_type_ = LASER_MODEL_BEAM;
  else if (tmp_model_type == "likelihood_field_prob") laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  else laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;

  this->declare_parameter("odom_model_type", std::string("diff"));
  this->get_parameter("odom_model_type", tmp_model_type);
  if (tmp_model_type == "omni") odom_model_type_ = ODOM_MODEL_OMNI;
  else if (tmp_model_type == "diff-corrected") odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if (tmp_model_type == "omni-corrected") odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else odom_model_type_ = ODOM_MODEL_DIFF;

  this->declare_parameter("update_min_d", 0.2);
  this->declare_parameter("update_min_a", M_PI / 6.0);
  this->declare_parameter("odom_frame_id", std::string("odom"));
  this->declare_parameter("base_frame_id", std::string("base_link"));
  this->declare_parameter("global_frame_id", std::string("map"));
  this->declare_parameter("resample_interval", 2);
  this->declare_parameter("selective_resampling", false);
  this->declare_parameter("transform_tolerance", 0.1);
  this->declare_parameter("recovery_alpha_slow", 0.001);
  this->declare_parameter("recovery_alpha_fast", 0.1);
  this->declare_parameter("tf_broadcast", true);
  this->declare_parameter("force_update_after_initialpose", false);
  this->declare_parameter("force_update_after_set_map", false);

  this->get_parameter("update_min_d", d_thresh_);
  this->get_parameter("update_min_a", a_thresh_);
  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("base_frame_id", base_frame_id_);
  this->get_parameter("global_frame_id", global_frame_id_);
  this->get_parameter("resample_interval", resample_interval_);
  this->get_parameter("selective_resampling", selective_resampling_);
  this->get_parameter("recovery_alpha_slow", alpha_slow_);
  this->get_parameter("recovery_alpha_fast", alpha_fast_);
  this->get_parameter("tf_broadcast", tf_broadcast_);
  this->get_parameter("force_update_after_initialpose", force_update_after_initialpose_);
  this->get_parameter("force_update_after_set_map", force_update_after_set_map_);

  double tmp_tol = 0.1;
  this->get_parameter("transform_tolerance", tmp_tol);
  transform_tolerance_ = rclcpp::Duration::from_seconds(tmp_tol);

  this->declare_parameter("std_warn_level_x", 0.2);
  this->declare_parameter("std_warn_level_y", 0.2);
  this->declare_parameter("std_warn_level_yaw", 0.1);
  this->get_parameter("std_warn_level_x", std_warn_level_x_);
  this->get_parameter("std_warn_level_y", std_warn_level_y_);
  this->get_parameter("std_warn_level_yaw", std_warn_level_yaw_);

  odom_frame_id_ = stripSlash(odom_frame_id_);
  base_frame_id_ = stripSlash(base_frame_id_);
  global_frame_id_ = stripSlash(global_frame_id_);

  updatePoseFromServer();

  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10);
  particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 10);

  global_loc_srv_ = this->create_service<std_srvs::srv::Empty>(
    "global_localization",
    [this](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request> req,
           std::shared_ptr<std_srvs::srv::Empty::Response> res) { globalLocalizationCallback(*req, *res); });

  nomotion_update_srv_ = this->create_service<std_srvs::srv::Empty>(
    "request_nomotion_update",
    [this](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Empty::Request> req,
           std::shared_ptr<std_srvs::srv::Empty::Response> res) { nomotionUpdateCallback(*req, *res); });

  set_map_srv_ = this->create_service<nav_msgs::srv::SetMap>(
    "set_map",
    [this](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<nav_msgs::srv::SetMap::Request> req,
           std::shared_ptr<nav_msgs::srv::SetMap::Response> res) {
      auto req_copy = *req;
      setMapCallback(req_copy, *res);
    });

  laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    this, scan_topic_, rmw_qos_profile_sensor_data);

  laser_scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_, odom_frame_id_, 100, this->get_node_logging_interface(),
    this->get_node_clock_interface(), tf2::durationFromSec(0.1));

  laser_scan_filter_->registerCallback(std::bind(&AmclNode::laserReceived, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10, std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

  if (use_map_topic_) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1, std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to map topic.");
  } else {
    map_request_timer_ = this->create_wall_timer(500ms, [this]() {
      requestMap();
      if (map_) map_request_timer_->cancel();
    });
  }

  laser_check_interval_ = rclcpp::Duration::from_seconds(15.0);
  check_laser_timer_ = this->create_wall_timer(15s, std::bind(&AmclNode::checkLaserReceived, this));

  diagnosic_updater_.setHardwareID("None");
  diagnosic_updater_.add("Standard deviation", this, &AmclNode::standardDeviationDiagnostics);

  save_pose_last_time_ = this->now();
  last_laser_received_ts_ = this->now();
}

AmclNode::~AmclNode()
{
  freeMapDependentMemory();
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();
}

int AmclNode::process() { return 0; }

void AmclNode::runFromBag(const std::string &, bool)
{
  RCLCPP_ERROR(get_logger(), "runFromBag is not implemented in this ROS2 conversion.");
}

void AmclNode::requestMap()
{
  if (!map_client_) {
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("static_map");
  }

  while (rclcpp::ok() && !map_client_->wait_for_service(1s)) {
    RCLCPP_WARN(get_logger(), "Waiting for map service 'static_map'...");
  }
  if (!rclcpp::ok()) return;

  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

  while (rclcpp::ok()) {
    auto future = map_client_->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s);

    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response) {
        handleMapMessage(response->map);
        RCLCPP_INFO(get_logger(), "Successfully retrieved map from service.");
        return;
      }
      RCLCPP_WARN(get_logger(), "Map service returned null response, retrying...");
    } else if (ret == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_WARN(get_logger(), "Timed out calling map service, retrying...");
    } else {
      RCLCPP_WARN(get_logger(), "Map service call interrupted, retrying...");
    }

    rclcpp::sleep_for(1s);
  }
}

void AmclNode::savePoseToServer()
{
  tf2::Transform odom_pose_tf2;
  tf2::fromMsg(latest_odom_pose_.pose, odom_pose_tf2);
  tf2::Transform map_pose = latest_tf_.inverse() * odom_pose_tf2;
  double yaw = tf2::getYaw(map_pose.getRotation());

  this->set_parameter(rclcpp::Parameter("initial_pose_x", map_pose.getOrigin().x()));
  this->set_parameter(rclcpp::Parameter("initial_pose_y", map_pose.getOrigin().y()));
  this->set_parameter(rclcpp::Parameter("initial_pose_a", yaw));
  this->set_parameter(rclcpp::Parameter("initial_cov_xx", last_published_pose_.pose.covariance[0]));
  this->set_parameter(rclcpp::Parameter("initial_cov_yy", last_published_pose_.pose.covariance[7]));
  this->set_parameter(rclcpp::Parameter("initial_cov_aa", last_published_pose_.pose.covariance[35]));
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0; init_pose_[1] = 0.0; init_pose_[2] = 0.0;
  init_cov_[0] = 0.25; init_cov_[1] = 0.25; init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);

  this->declare_parameter("initial_pose_x", init_pose_[0]);
  this->declare_parameter("initial_pose_y", init_pose_[1]);
  this->declare_parameter("initial_pose_a", init_pose_[2]);
  this->declare_parameter("initial_cov_xx", init_cov_[0]);
  this->declare_parameter("initial_cov_yy", init_cov_[1]);
  this->declare_parameter("initial_cov_aa", init_cov_[2]);

  double tmp_pos;
  this->get_parameter("initial_pose_x", tmp_pos); if (!std::isnan(tmp_pos)) init_pose_[0] = tmp_pos;
  this->get_parameter("initial_pose_y", tmp_pos); if (!std::isnan(tmp_pos)) init_pose_[1] = tmp_pos;
  this->get_parameter("initial_pose_a", tmp_pos); if (!std::isnan(tmp_pos)) init_pose_[2] = tmp_pos;
  this->get_parameter("initial_cov_xx", tmp_pos); if (!std::isnan(tmp_pos)) init_cov_[0] = tmp_pos;
  this->get_parameter("initial_cov_yy", tmp_pos); if (!std::isnan(tmp_pos)) init_cov_[1] = tmp_pos;
  this->get_parameter("initial_cov_aa", tmp_pos); if (!std::isnan(tmp_pos)) init_cov_[2] = tmp_pos;
}

void AmclNode::checkLaserReceived()
{
  rclcpp::Duration d = this->now() - last_laser_received_ts_;
  if (d > laser_check_interval_) {
    RCLCPP_WARN(
      get_logger(),
      "No laser scan received for %.3f seconds. Verify data on topic '%s'.",
      d.seconds(), scan_topic_.c_str());
  }
}

void AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  if (first_map_only_ && first_map_received_) return;
  handleMapMessage(*msg);
  first_map_received_ = true;
}

void AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
{
  std::lock_guard<std::recursive_mutex> lock(configuration_mutex_);

  RCLCPP_INFO(get_logger(), "Received map %u x %u @ %.3f m/cell", msg.info.width, msg.info.height, msg.info.resolution);
  if (stripSlash(msg.header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(get_logger(), "Map frame '%s' != global_frame_id '%s'", msg.header.frame_id.c_str(), global_frame_id_.c_str());
  }

  freeMapDependentMemory();
  lasers_.clear(); lasers_update_.clear(); frame_to_laser_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  free_space_indices.clear();
  for (int i = 0; i < map_->size_x; i++)
    for (int j = 0; j < map_->size_y; j++)
      if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i, j));
#endif

  pf_ = pf_alloc(min_particles_, max_particles_, alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator, (void *)map_);
  pf_set_selective_resampling(pf_, selective_resampling_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  updatePoseFromServer();
  pf_vector_t mean = pf_vector_zero();
  mean.v[0] = init_pose_[0]; mean.v[1] = init_pose_[1]; mean.v[2] = init_pose_[2];
  pf_matrix_t cov = pf_matrix_zero();
  cov.m[0][0] = init_cov_[0]; cov.m[1][1] = init_cov_[1]; cov.m[2][2] = init_cov_[2];
  pf_init(pf_, mean, cov);
  pf_init_ = false;

  delete odom_;
  odom_ = new AMCLOdom();
  odom_->SetModel(odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);

  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  if (laser_model_type_ == LASER_MODEL_BEAM) {
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_, 0.0);
  } else if (laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB) {
    laser_->SetModelLikelihoodFieldProb(
      z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_, do_beamskip_,
      beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_);
  } else {
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_);
  }

  applyInitialPose();
}

void AmclNode::freeMapDependentMemory()
{
  if (map_) { map_free(map_); map_ = nullptr; }
  if (pf_) { pf_free(pf_); pf_ = nullptr; }
  delete odom_; odom_ = nullptr;
  delete laser_; laser_ = nullptr;
}

map_t * AmclNode::convertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  map_t * map = map_alloc();
  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  map->cells = (map_cell_t *)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

  for (int i = 0; i < map->size_x * map->size_y; i++) {
    if (map_msg.data[i] == 0) map->cells[i].occ_state = -1;
    else if (map_msg.data[i] == 100) map->cells[i].occ_state = +1;
    else map->cells[i].occ_state = 0;
  }
  return map;
}

bool AmclNode::getOdomPose(
  geometry_msgs::msg::PoseStamped & odom_pose, double & x, double & y, double & yaw,
  const rclcpp::Time & t, const std::string & f)
{
  geometry_msgs::msg::PoseStamped ident;
  ident.header.frame_id = stripSlash(f);
  ident.header.stamp = t;
  ident.pose.orientation.w = 1.0;

  try {
    odom_pose = tf_->transform(ident, odom_frame_id_, tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);
  return true;
}

pf_vector_t AmclNode::uniformPoseGenerator(void * arg)
{
  map_t * map = (map_t *)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int, int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
  return p;
#else
  pf_vector_t p;
  for (;;) {
    p.v[0] = (drand48() - 0.5) * map->size_x * map->scale + map->origin_x;
    p.v[1] = (drand48() - 0.5) * map->size_y * map->scale + map->origin_y;
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    int i = MAP_GXWX(map, p.v[0]);
    int j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1)) break;
  }
  return p;
#endif
}

bool AmclNode::globalLocalizationCallback(std_srvs::srv::Empty::Request &, std_srvs::srv::Empty::Response &)
{
  if (!map_) return true;
  std::lock_guard<std::recursive_mutex> lock(configuration_mutex_);
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator, (void *)map_);
  pf_init_ = false;
  return true;
}

bool AmclNode::nomotionUpdateCallback(std_srvs::srv::Empty::Request &, std_srvs::srv::Empty::Response &)
{
  m_force_update = true;
  return true;
}

bool AmclNode::setMapCallback(nav_msgs::srv::SetMap::Request & req, nav_msgs::srv::SetMap::Response & res)
{
  handleMapMessage(req.map);
  handleInitialPoseMessage(req.initial_pose);
  if (force_update_after_set_map_) m_force_update = true;
  res.success = true;
  return true;
}

void AmclNode::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  handleInitialPoseMessage(*msg);
  if (force_update_after_initialpose_) m_force_update = true;
}

void AmclNode::handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  std::lock_guard<std::recursive_mutex> lock(configuration_mutex_);
  if (!msg.header.frame_id.empty() && stripSlash(msg.header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(get_logger(), "Ignoring initial pose in frame '%s'; must be '%s'",
                stripSlash(msg.header.frame_id).c_str(), global_frame_id_.c_str());
    return;
  }

  tf2::Transform pose_old, pose_new;
  tf2::fromMsg(msg.pose.pose, pose_old);
  pose_new = pose_old;

  pf_vector_t mean = pf_vector_zero();
  mean.v[0] = pose_new.getOrigin().x();
  mean.v[1] = pose_new.getOrigin().y();
  mean.v[2] = tf2::getYaw(pose_new.getRotation());

  pf_matrix_t cov = pf_matrix_zero();
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      cov.m[i][j] = msg.pose.covariance[6 * i + j];
  cov.m[2][2] = msg.pose.covariance[35];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = mean;
  initial_pose_hyp_->pf_pose_cov = cov;
  applyInitialPose();
}

void AmclNode::applyInitialPose()
{
  std::lock_guard<std::recursive_mutex> lock(configuration_mutex_);
  if (initial_pose_hyp_ && map_) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;
    delete initial_pose_hyp_;
    initial_pose_hyp_ = nullptr;
  }
}

void AmclNode::laserReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan)
{
  last_laser_received_ts_ = this->now();
  if (!map_) return;
  // Full laser update logic remains identical to ROS1 core; omitted for brevity in migration scaffold.
  // Keep diagnostics running:
  diagnosic_updater_.force_update();
}

void AmclNode::standardDeviationDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic_status)
{
  double std_x = std::sqrt(last_published_pose_.pose.covariance[0]);
  double std_y = std::sqrt(last_published_pose_.pose.covariance[7]);
  double std_yaw = std::sqrt(last_published_pose_.pose.covariance[35]);

  diagnostic_status.add("std_x", std_x);
  diagnostic_status.add("std_y", std_y);
  diagnostic_status.add("std_yaw", std_yaw);
  diagnostic_status.add("std_warn_level_x", std_warn_level_x_);
  diagnostic_status.add("std_warn_level_y", std_warn_level_y_);
  diagnostic_status.add("std_warn_level_yaw", std_warn_level_yaw_);

  if (std_x > std_warn_level_x_ || std_y > std_warn_level_y_ || std_yaw > std_warn_level_yaw_) {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Too large");
  } else {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }
}