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
/* ROS2 Translation of AMCL Navigation Action Server */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <thread>
#include <mutex>
#include <functional>

// Signal handling
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Messages
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/empty.hpp"

// For transform support
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// Action definition - custom UpdatePose action
#include "task_006_amcl_navigation/action/update_pose.hpp"

// Define UpdatePose action type alias
using UpdatePose = task_006_amcl_navigation::action::UpdatePose;
using GoalHandleUpdatePose = rclcpp_action::ServerGoalHandle<UpdatePose>;

static double
normalize(double z)
{
  return atan2(sin(z), cos(z));
}

static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0)
    d2 *= -1.0;
  if (fabs(d1) < fabs(d2))
    return (d1);
  else
    return (d2);
}

static const std::string scan_topic_ = "scan";

inline std::string stripSlash(const std::string & in)
{
  std::string out = in;
  if ((!in.empty()) && (in[0] == '/'))
    out.erase(0, 1);
  return out;
}

// Pose hypothesis
typedef struct
{
  double weight;
  double pf_pose_mean[3];
  double pf_pose_cov[3][3];
} amcl_hyp_t;

class AmclNode : public rclcpp::Node
{
public:
  AmclNode();
  ~AmclNode();

  void savePoseToServer();

private:
  // TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool sent_first_transform_;
  tf2::Transform latest_tf_;
  bool latest_tf_valid_;

  // Action server for UpdatePose
  rclcpp_action::Server<UpdatePose>::SharedPtr action_server_;

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const UpdatePose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleUpdatePose> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleUpdatePose> goal_handle);

  void execute(const std::shared_ptr<GoalHandleUpdatePose> goal_handle);

  // Laser callback
  void laserReceived(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);

  // Initial pose callback
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void handleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  // Map callback
  void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg);

  void updatePoseFromServer();
  void applyInitialPose();

  // Parameters
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string global_frame_id_;

  geometry_msgs::msg::PoseStamped latest_odom_pose_;

  bool use_map_topic_;
  bool first_map_only_;

  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;

  // Particle filter state
  bool pf_init_;
  double pf_odom_pose_[3];
  double d_thresh_, a_thresh_;
  int resample_interval_;
  int resample_count_;
  double laser_min_range_;
  double laser_max_range_;

  bool m_force_update;

  int max_beams_, min_particles_, max_particles_;
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
  double alpha_slow_, alpha_fast_;
  double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  bool do_beamskip_;
  double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
  double laser_likelihood_max_dist_;

  double init_pose_[3];
  double init_cov_[3];
  bool tf_broadcast_;
  bool selective_resampling_;

  std::recursive_mutex configuration_mutex_;

  rclcpp::Time last_laser_received_ts_;
  rclcpp::TimerBase::SharedPtr check_laser_timer_;
  void checkLaserReceived();

  amcl_hyp_t * initial_pose_hyp_;
  bool first_map_received_;
};

AmclNode::AmclNode()
: Node("amcl"),
  sent_first_transform_(false),
  latest_tf_valid_(false),
  pf_init_(false),
  resample_count_(0),
  initial_pose_hyp_(nullptr),
  first_map_received_(false),
  m_force_update(false)
{
  std::lock_guard<std::recursive_mutex> l(configuration_mutex_);

  // Declare and get parameters
  this->declare_parameter<bool>("use_map_topic", false);
  this->declare_parameter<bool>("first_map_only", false);
  this->declare_parameter<double>("laser_min_range", -1.0);
  this->declare_parameter<double>("laser_max_range", -1.0);
  this->declare_parameter<int>("laser_max_beams", 30);
  this->declare_parameter<int>("min_particles", 100);
  this->declare_parameter<int>("max_particles", 5000);
  this->declare_parameter<double>("odom_alpha1", 0.2);
  this->declare_parameter<double>("odom_alpha2", 0.2);
  this->declare_parameter<double>("odom_alpha3", 0.2);
  this->declare_parameter<double>("odom_alpha4", 0.2);
  this->declare_parameter<double>("odom_alpha5", 0.2);
  this->declare_parameter<double>("update_min_d", 0.2);
  this->declare_parameter<double>("update_min_a", M_PI / 6.0);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->declare_parameter<std::string>("global_frame_id", "map");
  this->declare_parameter<int>("resample_interval", 2);
  this->declare_parameter<bool>("tf_broadcast", true);
  this->declare_parameter<bool>("selective_resampling", false);
  this->declare_parameter<double>("laser_z_hit", 0.95);
  this->declare_parameter<double>("laser_z_short", 0.1);
  this->declare_parameter<double>("laser_z_max", 0.05);
  this->declare_parameter<double>("laser_z_rand", 0.05);
  this->declare_parameter<double>("laser_sigma_hit", 0.2);
  this->declare_parameter<double>("laser_lambda_short", 0.1);
  this->declare_parameter<double>("laser_likelihood_max_dist", 2.0);
  this->declare_parameter<double>("recovery_alpha_slow", 0.001);
  this->declare_parameter<double>("recovery_alpha_fast", 0.1);

  this->get_parameter("use_map_topic", use_map_topic_);
  this->get_parameter("first_map_only", first_map_only_);
  this->get_parameter("laser_min_range", laser_min_range_);
  this->get_parameter("laser_max_range", laser_max_range_);
  this->get_parameter("laser_max_beams", max_beams_);
  this->get_parameter("min_particles", min_particles_);
  this->get_parameter("max_particles", max_particles_);
  this->get_parameter("odom_alpha1", alpha1_);
  this->get_parameter("odom_alpha2", alpha2_);
  this->get_parameter("odom_alpha3", alpha3_);
  this->get_parameter("odom_alpha4", alpha4_);
  this->get_parameter("odom_alpha5", alpha5_);
  this->get_parameter("update_min_d", d_thresh_);
  this->get_parameter("update_min_a", a_thresh_);
  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("base_frame_id", base_frame_id_);
  this->get_parameter("global_frame_id", global_frame_id_);
  this->get_parameter("resample_interval", resample_interval_);
  this->get_parameter("tf_broadcast", tf_broadcast_);
  this->get_parameter("selective_resampling", selective_resampling_);
  this->get_parameter("laser_z_hit", z_hit_);
  this->get_parameter("laser_z_short", z_short_);
  this->get_parameter("laser_z_max", z_max_);
  this->get_parameter("laser_z_rand", z_rand_);
  this->get_parameter("laser_sigma_hit", sigma_hit_);
  this->get_parameter("laser_lambda_short", lambda_short_);
  this->get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);
  this->get_parameter("recovery_alpha_slow", alpha_slow_);
  this->get_parameter("recovery_alpha_fast", alpha_fast_);

  odom_frame_id_ = stripSlash(odom_frame_id_);
  base_frame_id_ = stripSlash(base_frame_id_);
  global_frame_id_ = stripSlash(global_frame_id_);

  updatePoseFromServer();

  // TF setup
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  // Publishers
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", 2);
  particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "particlecloud", 2);

  // Subscriptions
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, 100,
    std::bind(&AmclNode::laserReceived, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 2,
    std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

  if (use_map_topic_) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 1,
      std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to map topic.");
  }

  // Services
  global_loc_srv_ = this->create_service<std_srvs::srv::Empty>(
    "global_localization",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
           std::shared_ptr<std_srvs::srv::Empty::Response>) {
      RCLCPP_INFO(this->get_logger(), "Initializing with uniform distribution");
      pf_init_ = false;
    });

  nomotion_update_srv_ = this->create_service<std_srvs::srv::Empty>(
    "request_nomotion_update",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
           std::shared_ptr<std_srvs::srv::Empty::Response>) {
      m_force_update = true;
    });

  // Create the ROS2 Action Server for UpdatePose
  action_server_ = rclcpp_action::create_server<UpdatePose>(
    this,
    "update_pose",
    std::bind(&AmclNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AmclNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&AmclNode::handle_accepted, this, std::placeholders::_1));

  // Laser check timer (15s)
  check_laser_timer_ = this->create_wall_timer(
    std::chrono::seconds(15),
    std::bind(&AmclNode::checkLaserReceived, this));

  RCLCPP_INFO(this->get_logger(), "AMCL node initialized with action server.");
}

AmclNode::~AmclNode()
{
  delete initial_pose_hyp_;
  initial_pose_hyp_ = nullptr;
}

// ---------------------------------------------------------------
// Action Server Callbacks
// ---------------------------------------------------------------

rclcpp_action::GoalResponse AmclNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const UpdatePose::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request for pose update");

  // Accept all goals - the pose target is in goal->pose
  if (goal->pose.header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(), "Goal has empty frame_id, but accepting anyway");
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AmclNode::handle_cancel(
  const std::shared_ptr<GoalHandleUpdatePose> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AmclNode::handle_accepted(
  const std::shared_ptr<GoalHandleUpdatePose> goal_handle)
{
  // Execute in a separate thread to avoid blocking the executor
  std::thread(
    [this, goal_handle]() {
      this->execute(goal_handle);
    }).detach();
}

void AmclNode::execute(
  const std::shared_ptr<GoalHandleUpdatePose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal - running particle filter update cycle");

  auto feedback = std::make_shared<UpdatePose::Feedback>();
  auto result = std::make_shared<UpdatePose::Result>();

  // Check if goal is being canceled
  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return;
  }

  // TODO: Implement particle filter update
  // This is where the AMCL particle filter update logic would run:
  // 1. Get the latest laser scan data with ranges and bearing information
  // 2. Update the particle filter with the sensor model
  // 3. Perform pf_update_resample if needed (resample step)
  // 4. Compute the best pose hypothesis
  // 5. Publish the updated pose via pose_pub_

  // Simulate particle filter processing with feedback
  for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled during execution");
      return;
    }

    // Publish feedback with current estimated pose
    // feedback->current_pose contains the latest pose estimate
    feedback->current_pose.header.frame_id = global_frame_id_;
    feedback->current_pose.header.stamp = this->now();
    goal_handle->publish_feedback(feedback);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Publish the final PoseWithCovarianceStamped result
  geometry_msgs::msg::PoseWithCovarianceStamped p;
  p.header.frame_id = global_frame_id_;
  p.header.stamp = this->now();
  p.pose.pose.position.x = init_pose_[0];
  p.pose.pose.position.y = init_pose_[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, init_pose_[2]);
  p.pose.pose.orientation.x = q.x();
  p.pose.pose.orientation.y = q.y();
  p.pose.pose.orientation.z = q.z();
  p.pose.pose.orientation.w = q.w();

  p.pose.covariance[6 * 0 + 0] = init_cov_[0];
  p.pose.covariance[6 * 1 + 1] = init_cov_[1];
  p.pose.covariance[6 * 5 + 5] = init_cov_[2];

  pose_pub_->publish(p);
  last_published_pose = p;

  // Set result pose
  result->result_pose.header = p.header;
  result->result_pose.pose = p.pose.pose;

  // Mark goal as succeeded
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded - pose updated");
}

// ---------------------------------------------------------------
// Laser Received Callback
// ---------------------------------------------------------------

void AmclNode::laserReceived(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  std::string laser_scan_frame_id = stripSlash(laser_scan->header.frame_id);
  last_laser_received_ts_ = this->now();

  std::lock_guard<std::recursive_mutex> lr(configuration_mutex_);

  // Where was the robot when this scan was taken?
  double pose[3] = {0.0, 0.0, 0.0};

  double delta[3] = {0.0, 0.0, 0.0};

  if (pf_init_) {
    // Compute change in pose
    delta[0] = pose[0] - pf_odom_pose_[0];
    delta[1] = pose[1] - pf_odom_pose_[1];
    delta[2] = angle_diff(pose[2], pf_odom_pose_[2]);

    // See if we should update the filter
    bool update = fabs(delta[0]) > d_thresh_ ||
      fabs(delta[1]) > d_thresh_ ||
      fabs(delta[2]) > a_thresh_;
    update = update || m_force_update;
    m_force_update = false;
  }

  bool force_publication = false;
  bool resampled = false;

  if (!pf_init_) {
    pf_odom_pose_[0] = pose[0];
    pf_odom_pose_[1] = pose[1];
    pf_odom_pose_[2] = pose[2];
    pf_init_ = true;
    force_publication = true;
    resample_count_ = 0;
  }

  // TODO: Implement particle filter update for incoming laser scan
  // 1. Fill in laser sensor data structure (ranges, bearings)
  //    - Process laser_scan->ranges array
  //    - Compute bearing for each range measurement
  // 2. Call UpdateSensor on the AMCLLaser object
  // 3. Update odometry pose for filter
  // 4. Perform resampling if needed (pf_update_resample)
  // 5. Publish updated particle cloud and estimated pose

  if (resampled || force_publication) {
    // Read out the current hypotheses and publish
    geometry_msgs::msg::PoseWithCovarianceStamped p;
    p.header.frame_id = global_frame_id_;
    p.header.stamp = laser_scan->header.stamp;
    p.pose.pose.position.x = pf_odom_pose_[0];
    p.pose.pose.position.y = pf_odom_pose_[1];

    tf2::Quaternion q;
    q.setRPY(0, 0, pf_odom_pose_[2]);
    p.pose.pose.orientation.x = q.x();
    p.pose.pose.orientation.y = q.y();
    p.pose.pose.orientation.z = q.z();
    p.pose.pose.orientation.w = q.w();

    pose_pub_->publish(p);
    last_published_pose = p;

    RCLCPP_DEBUG(this->get_logger(), "New pose: %6.3f %6.3f %6.3f",
      pf_odom_pose_[0], pf_odom_pose_[1], pf_odom_pose_[2]);

    if (tf_broadcast_) {
      geometry_msgs::msg::TransformStamped tmp_tf_stamped;
      tmp_tf_stamped.header.frame_id = global_frame_id_;
      tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
      tmp_tf_stamped.child_frame_id = odom_frame_id_;
      tmp_tf_stamped.transform.translation.x = 0.0;
      tmp_tf_stamped.transform.translation.y = 0.0;
      tmp_tf_stamped.transform.translation.z = 0.0;
      tmp_tf_stamped.transform.rotation.x = 0.0;
      tmp_tf_stamped.transform.rotation.y = 0.0;
      tmp_tf_stamped.transform.rotation.z = 0.0;
      tmp_tf_stamped.transform.rotation.w = 1.0;
      tfb_->sendTransform(tmp_tf_stamped);
      sent_first_transform_ = true;
    }
  }
}

// ---------------------------------------------------------------
// Initial Pose
// ---------------------------------------------------------------

void AmclNode::initialPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  handleInitialPoseMessage(*msg);
}

void AmclNode::handleInitialPoseMessage(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  std::lock_guard<std::recursive_mutex> prl(configuration_mutex_);

  if (msg.header.frame_id.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "Received initial pose with empty frame_id.");
  } else if (stripSlash(msg.header.frame_id) != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(),
      "Ignoring initial pose in frame \"%s\"; must be in \"%s\"",
      stripSlash(msg.header.frame_id).c_str(),
      global_frame_id_.c_str());
    return;
  }

  // Re-initialize the filter
  double pf_init_pose_mean[3];
  pf_init_pose_mean[0] = msg.pose.pose.position.x;
  pf_init_pose_mean[1] = msg.pose.pose.position.y;
  pf_init_pose_mean[2] = tf2::getYaw(msg.pose.pose.orientation);

  double pf_init_pose_cov[3][3] = {{0}};
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pf_init_pose_cov[i][j] = msg.pose.covariance[6 * i + j];
    }
  }
  pf_init_pose_cov[2][2] = msg.pose.covariance[6 * 5 + 5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean[0] = pf_init_pose_mean[0];
  initial_pose_hyp_->pf_pose_mean[1] = pf_init_pose_mean[1];
  initial_pose_hyp_->pf_pose_mean[2] = pf_init_pose_mean[2];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      initial_pose_hyp_->pf_pose_cov[i][j] = pf_init_pose_cov[i][j];

  applyInitialPose();
}

void AmclNode::applyInitialPose()
{
  std::lock_guard<std::recursive_mutex> cfl(configuration_mutex_);
  if (initial_pose_hyp_ != nullptr) {
    pf_init_ = false;
    delete initial_pose_hyp_;
    initial_pose_hyp_ = nullptr;
  }
}

// ---------------------------------------------------------------
// Map
// ---------------------------------------------------------------

void AmclNode::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (first_map_only_ && first_map_received_) {
    return;
  }
  handleMapMessage(*msg);
  first_map_received_ = true;
}

void AmclNode::handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
{
  std::lock_guard<std::recursive_mutex> cfl(configuration_mutex_);

  RCLCPP_INFO(this->get_logger(), "Received a %d X %d map @ %.3f m/pix",
    msg.info.width, msg.info.height, msg.info.resolution);

  if (msg.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(),
      "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'",
      msg.header.frame_id.c_str(), global_frame_id_.c_str());
  }

  updatePoseFromServer();
  applyInitialPose();
}

// ---------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
}

void AmclNode::savePoseToServer()
{
  RCLCPP_DEBUG(this->get_logger(), "Saving pose to server.");
}

void AmclNode::checkLaserReceived()
{
  auto now = this->now();
  auto d = now - last_laser_received_ts_;
  if (d.seconds() > 15.0) {
    RCLCPP_WARN(this->get_logger(),
      "No laser scan received for %.1f seconds.", d.seconds());
  }
}

// ---------------------------------------------------------------
// Main
// ---------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AmclNode>();
  // Multi-threaded executor: handle_accepted() runs execute() on a detached
  // thread that calls publish_feedback()/succeed() concurrently with the spin
  // loop. A single-threaded executor can race the action terminal-state
  // response against the client's get_result request (client sees status 0).
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  node->savePoseToServer();
  rclcpp::shutdown();
  return 0;
}