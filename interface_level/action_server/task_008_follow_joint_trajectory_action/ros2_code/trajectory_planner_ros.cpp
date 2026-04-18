/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <base_local_planner/trajectory_planner_ros.h>

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <boost/tokenizer.hpp>

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/msg/path.hpp>

#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace base_local_planner
{

void TrajectoryPlannerROS::reconfigureCB(BaseLocalPlannerConfig & config, uint32_t level)
{
  (void)level;
  if (setup_ && config.restore_defaults) {
    config = default_config_;
    config.restore_defaults = false;
  }
  if (!setup_) {
    default_config_ = config;
    setup_ = true;
  }
  tc_->reconfigure(config);
  reached_goal_ = false;
}

TrajectoryPlannerROS::TrajectoryPlannerROS()
: world_model_(nullptr), tc_(nullptr), costmap_ros_(nullptr), tf_(nullptr),
  setup_(false), initialized_(false), odom_helper_("odom")
{
}

TrajectoryPlannerROS::TrajectoryPlannerROS(
  std::string name, tf2_ros::Buffer * tf, costmap_2d::Costmap2DROS * costmap_ros)
: world_model_(nullptr), tc_(nullptr), costmap_ros_(nullptr), tf_(nullptr),
  setup_(false), initialized_(false), odom_helper_("odom")
{
  initialize(name, tf, costmap_ros);
}

void TrajectoryPlannerROS::initialize(
  std::string name,
  tf2_ros::Buffer * tf,
  costmap_2d::Costmap2DROS * costmap_ros)
{
  if (isInitialized()) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryPlannerROS"), "This planner has already been initialized, doing nothing");
    return;
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  auto node = costmap_ros_->getNode();

  g_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name + "/global_plan", rclcpp::QoS(1));
  l_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name + "/local_plan", rclcpp::QoS(1));

  auto declare_or_get = [&](const std::string & param, auto default_value) {
      using T = decltype(default_value);
      const std::string scoped = name + "." + param;
      if (!node->has_parameter(scoped)) {
        node->declare_parameter<T>(scoped, default_value);
      }
      T value = default_value;
      node->get_parameter(scoped, value);
      return value;
    };

  rot_stopped_velocity_ = 1e-2;
  trans_stopped_velocity_ = 1e-2;

  double sim_time, sim_granularity, angular_sim_granularity;
  int vx_samples, vtheta_samples;
  double path_distance_bias, goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist,
    escape_reset_dist, escape_reset_theta;
  bool holonomic_robot, dwa, simple_attractor, heading_scoring;
  double heading_scoring_timestep;
  double max_vel_x, min_vel_x;
  double backup_vel;
  double stop_time_buffer;
  std::string world_model_type;
  rotating_to_goal_ = false;

  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  robot_base_frame_ = costmap_ros_->getBaseFrameID();

  prune_plan_ = declare_or_get("prune_plan", true);
  yaw_goal_tolerance_ = declare_or_get("yaw_goal_tolerance", 0.05);
  xy_goal_tolerance_ = declare_or_get("xy_goal_tolerance", 0.10);
  acc_lim_x_ = declare_or_get("acc_lim_x", 2.5);
  acc_lim_y_ = declare_or_get("acc_lim_y", 2.5);
  acc_lim_theta_ = declare_or_get("acc_lim_theta", 3.2);

  stop_time_buffer = declare_or_get("stop_time_buffer", 0.2);
  latch_xy_goal_tolerance_ = declare_or_get("latch_xy_goal_tolerance", false);

  if (node->has_parameter(name + ".acc_limit_x")) {
    RCLCPP_ERROR(
      node->get_logger(),
      "You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately.");
  }
  if (node->has_parameter(name + ".acc_limit_y")) {
    RCLCPP_ERROR(
      node->get_logger(),
      "You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately.");
  }
  if (node->has_parameter(name + ".acc_limit_th")) {
    RCLCPP_ERROR(
      node->get_logger(),
      "You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately.");
  }

  double controller_frequency = 20.0;
  if (node->has_parameter("controller_frequency")) {
    node->get_parameter("controller_frequency", controller_frequency);
  } else if (node->has_parameter(name + ".controller_frequency")) {
    node->get_parameter(name + ".controller_frequency", controller_frequency);
  }

  if (controller_frequency > 0.0) {
    sim_period_ = 1.0 / controller_frequency;
  } else {
    RCLCPP_WARN(node->get_logger(), "controller_frequency <= 0. Assuming 20Hz");
    sim_period_ = 0.05;
  }

  RCLCPP_INFO(node->get_logger(), "Sim period is set to %.2f", sim_period_);

  sim_time = declare_or_get("sim_time", 1.0);
  sim_granularity = declare_or_get("sim_granularity", 0.025);
  angular_sim_granularity = declare_or_get("angular_sim_granularity", sim_granularity);
  vx_samples = declare_or_get("vx_samples", 3);
  vtheta_samples = declare_or_get("vtheta_samples", 20);

  path_distance_bias = declare_or_get("path_distance_bias", 0.6);
  goal_distance_bias = declare_or_get("goal_distance_bias", 0.6);

  occdist_scale = declare_or_get("occdist_scale", 0.01);

  bool meter_scoring = false;
  meter_scoring = declare_or_get("meter_scoring", false);
  if (meter_scoring) {
    double resolution = costmap_->getResolution();
    goal_distance_bias *= resolution;
    path_distance_bias *= resolution;
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "Trajectory Rollout planner initialized with meter_scoring=false. Set true for resolution robustness.");
  }

  heading_lookahead = declare_or_get("heading_lookahead", 0.325);
  oscillation_reset_dist = declare_or_get("oscillation_reset_dist", 0.05);
  escape_reset_dist = declare_or_get("escape_reset_dist", 0.10);
  escape_reset_theta = declare_or_get("escape_reset_theta", M_PI_4);
  holonomic_robot = declare_or_get("holonomic_robot", true);
  max_vel_x = declare_or_get("max_vel_x", 0.5);
  min_vel_x = declare_or_get("min_vel_x", 0.1);

  double max_rotational_vel = declare_or_get("max_rotational_vel", 1.0);
  max_vel_th_ = max_rotational_vel;
  min_vel_th_ = -1.0 * max_rotational_vel;

  min_in_place_vel_th_ = declare_or_get("min_in_place_vel_theta", 0.4);
  reached_goal_ = false;

  backup_vel = declare_or_get("backup_vel", -0.1);
  if (node->has_parameter(name + ".backup_vel")) {
    RCLCPP_WARN(
      node->get_logger(),
      "backup_vel is deprecated in favor of escape_vel.");
  }
  if (node->has_parameter(name + ".escape_vel")) {
    node->get_parameter(name + ".escape_vel", backup_vel);
  }

  if (backup_vel >= 0.0) {
    RCLCPP_WARN(
      node->get_logger(),
      "You've specified a positive escape velocity. This usually should be negative.");
  }

  world_model_type = declare_or_get("world_model", std::string("costmap"));
  dwa = declare_or_get("dwa", true);
  heading_scoring = declare_or_get("heading_scoring", false);
  heading_scoring_timestep = declare_or_get("heading_scoring_timestep", 0.8);
  simple_attractor = false;

  double min_pt_separation, max_obstacle_height, grid_resolution;
  max_sensor_range_ = declare_or_get("point_grid.max_sensor_range", 2.0);
  min_pt_separation = declare_or_get("point_grid.min_pt_separation", 0.01);
  max_obstacle_height = declare_or_get("point_grid.max_obstacle_height", 2.0);
  grid_resolution = declare_or_get("point_grid.grid_resolution", 0.2);
  (void)min_pt_separation;
  (void)max_obstacle_height;
  (void)grid_resolution;

  if (world_model_type != "costmap") {
    throw std::runtime_error("Only costmap world models are supported by this controller");
  }

  world_model_ = new CostmapModel(*costmap_);
  std::vector<double> y_vels = loadYVels(node, name);

  footprint_spec_ = costmap_ros_->getRobotFootprint();

  tc_ = new TrajectoryPlanner(
    *world_model_, *costmap_, footprint_spec_,
    acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples,
    path_distance_bias, goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist,
    escape_reset_dist, escape_reset_theta, holonomic_robot, max_vel_x, min_vel_x, max_vel_th_, min_vel_th_,
    min_in_place_vel_th_, backup_vel, dwa, heading_scoring, heading_scoring_timestep, meter_scoring,
    simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

  map_viz_.initialize(
    name, global_frame_,
    [this](int cx, int cy, float & path_cost, float & goal_cost, float & occ_cost, float & total_cost) {
      return tc_->getCellCosts(cx, cy, path_cost, goal_cost, occ_cost, total_cost);
    });

  initialized_ = true;
}

std::vector<double> TrajectoryPlannerROS::loadYVels(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name)
{
  std::vector<double> y_vels;
  std::string y_vel_list;

  const std::string param = name + ".y_vels";
  if (!node->has_parameter(param)) {
    node->declare_parameter<std::string>(param, "");
  }
  node->get_parameter(param, y_vel_list);

  if (!y_vel_list.empty()) {
    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
    boost::char_separator<char> sep("[], ");
    tokenizer tokens(y_vel_list, sep);

    for (tokenizer::iterator i = tokens.begin(); i != tokens.end(); ++i) {
      y_vels.push_back(std::atof((*i).c_str()));
    }
  } else {
    y_vels.push_back(-0.3);
    y_vels.push_back(-0.1);
    y_vels.push_back(0.1);
    y_vels.push_back(0.3);
  }

  return y_vels;
}

TrajectoryPlannerROS::~TrajectoryPlannerROS()
{
  if (tc_ != nullptr) {
    delete tc_;
  }

  if (world_model_ != nullptr) {
    delete world_model_;
  }
}

bool TrajectoryPlannerROS::stopWithAccLimits(
  const geometry_msgs::msg::PoseStamped & global_pose,
  const geometry_msgs::msg::PoseStamped & robot_vel,
  geometry_msgs::msg::Twist & cmd_vel)
{
  double vx = sign(robot_vel.pose.position.x) *
    std::max(0.0, (std::fabs(robot_vel.pose.position.x) - acc_lim_x_ * sim_period_));
  double vy = sign(robot_vel.pose.position.y) *
    std::max(0.0, (std::fabs(robot_vel.pose.position.y) - acc_lim_y_ * sim_period_));

  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  double vth = sign(vel_yaw) *
    std::max(0.0, (std::fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  bool valid_cmd = tc_->checkTrajectory(
    global_pose.pose.position.x, global_pose.pose.position.y, yaw,
    robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, vx, vy, vth);

  if (valid_cmd) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("TrajectoryPlannerROS"),
      "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vth;
    return true;
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return false;
}

bool TrajectoryPlannerROS::rotateToGoal(
  const geometry_msgs::msg::PoseStamped & global_pose,
  const geometry_msgs::msg::PoseStamped & robot_vel, double goal_th,
  geometry_msgs::msg::Twist & cmd_vel)
{
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

  double v_theta_samp = ang_diff > 0.0 ?
    std::min(max_vel_th_, std::max(min_in_place_vel_th_, ang_diff)) :
    std::max(min_vel_th_, std::min(-1.0 * min_in_place_vel_th_, ang_diff));

  double max_acc_vel = std::fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
  double min_acc_vel = std::fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

  v_theta_samp = sign(v_theta_samp) * std::min(std::max(std::fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  double max_speed_to_stop = std::sqrt(2 * acc_lim_theta_ * std::fabs(ang_diff));
  v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, std::fabs(v_theta_samp));

  v_theta_samp = v_theta_samp > 0.0 ?
    std::min(max_vel_th_, std::max(min_in_place_vel_th_, v_theta_samp)) :
    std::max(min_vel_th_, std::min(-1.0 * min_in_place_vel_th_, v_theta_samp));

  bool valid_cmd = tc_->checkTrajectory(
    global_pose.pose.position.x, global_pose.pose.position.y, yaw,
    robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, 0.0, 0.0, v_theta_samp);

  RCLCPP_DEBUG(
    rclcpp::get_logger("TrajectoryPlannerROS"),
    "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d",
    v_theta_samp, static_cast<int>(valid_cmd));

  if (valid_cmd) {
    cmd_vel.angular.z = v_theta_samp;
    return true;
  }

  cmd_vel.angular.z = 0.0;
  return false;
}

bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::msg::PoseStamped> & orig_global_plan)
{
  if (!isInitialized()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TrajectoryPlannerROS"),
      "This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  global_plan_.clear();
  global_plan_ = orig_global_plan;

  xy_tolerance_latch_ = false;
  reached_goal_ = false;
  return true;
}

bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::msg::Twist & cmd_vel)
{
  if (!isInitialized()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TrajectoryPlannerROS"),
      "This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  std::vector<geometry_msgs::msg::PoseStamped> local_plan;
  geometry_msgs::msg::PoseStamped global_pose;
  if (!costmap_ros_->getRobotPose(global_pose)) {
    return false;
  }

  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
    RCLCPP_WARN(
      rclcpp::get_logger("TrajectoryPlannerROS"),
      "Could not transform the global plan to the frame of the controller");
    return false;
  }

  if (prune_plan_) {
    prunePlan(global_pose, transformed_plan, global_plan_);
  }

  geometry_msgs::msg::PoseStamped drive_cmds;
  drive_cmds.header.frame_id = robot_base_frame_;

  geometry_msgs::msg::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);

  if (transformed_plan.empty()) {
    return false;
  }

  const geometry_msgs::msg::PoseStamped & goal_point = transformed_plan.back();
  const double goal_x = goal_point.pose.position.x;
  const double goal_y = goal_point.pose.position.y;
  const double goal_th = tf2::getYaw(goal_point.pose.orientation);

  if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {
    if (latch_xy_goal_tolerance_) {
      xy_tolerance_latch_ = true;
    }

    double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
    if (std::fabs(angle) <= yaw_goal_tolerance_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      rotating_to_goal_ = false;
      xy_tolerance_latch_ = false;
      reached_goal_ = true;
    } else {
      tc_->updatePlan(transformed_plan);
      Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
      (void)path;
      map_viz_.publishCostCloud(costmap_);

      nav_msgs::msg::Odometry base_odom;
      odom_helper_.getOdom(base_odom);

      if (!rotating_to_goal_ &&
        !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_))
      {
        if (!stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
          return false;
        }
      } else {
        rotating_to_goal_ = true;
        if (!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
          return false;
        }
      }
    }

    publishPlan(transformed_plan, g_plan_pub_);
    publishPlan(local_plan, l_plan_pub_);
    return true;
  }

  tc_->updatePlan(transformed_plan);
  Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

  map_viz_.publishCostCloud(costmap_);

  cmd_vel.linear.x = drive_cmds.pose.position.x;
  cmd_vel.linear.y = drive_cmds.pose.position.y;
  cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

  if (path.cost_ < 0) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("trajectory_planner_ros"),
      "The rollout planner failed to find a valid plan.");
    local_plan.clear();
    publishPlan(transformed_plan, g_plan_pub_);
    publishPlan(local_plan, l_plan_pub_);
    return false;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("trajectory_planner_ros"),
    "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  auto node = costmap_ros_->getNode();
  for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
    double p_x, p_y, p_th;
    path.getPoint(i, p_x, p_y, p_th);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = node->now();
    pose.pose.position.x = p_x;
    pose.pose.position.y = p_y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, p_th);
    tf2::convert(q, pose.pose.orientation);
    local_plan.push_back(pose);
  }

  publishPlan(transformed_plan, g_plan_pub_);
  publishPlan(local_plan, l_plan_pub_);
  return true;
}

bool TrajectoryPlannerROS::checkTrajectory(
  double vx_samp, double vy_samp, double vtheta_samp, bool update_map)
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (costmap_ros_->getRobotPose(global_pose)) {
    if (update_map) {
      std::vector<geometry_msgs::msg::PoseStamped> plan;
      plan.push_back(global_pose);
      tc_->updatePlan(plan, true);
    }

    nav_msgs::msg::Odometry base_odom;
    {
      std::lock_guard<std::recursive_mutex> lock(odom_lock_);
      base_odom = base_odom_;
    }

    return tc_->checkTrajectory(
      global_pose.pose.position.x,
      global_pose.pose.position.y,
      tf2::getYaw(global_pose.pose.orientation),
      base_odom.twist.twist.linear.x,
      base_odom.twist.twist.linear.y,
      base_odom.twist.twist.angular.z,
      vx_samp,
      vy_samp,
      vtheta_samp);
  }

  RCLCPP_WARN(
    rclcpp::get_logger("TrajectoryPlannerROS"),
    "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
  return false;
}

double TrajectoryPlannerROS::scoreTrajectory(
  double vx_samp, double vy_samp, double vtheta_samp, bool update_map)
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (costmap_ros_->getRobotPose(global_pose)) {
    if (update_map) {
      std::vector<geometry_msgs::msg::PoseStamped> plan;
      plan.push_back(global_pose);
      tc_->updatePlan(plan, true);
    }

    nav_msgs::msg::Odometry base_odom;
    {
      std::lock_guard<std::recursive_mutex> lock(odom_lock_);
      base_odom = base_odom_;
    }

    return tc_->scoreTrajectory(
      global_pose.pose.position.x, global_pose.pose.position.y,
      tf2::getYaw(global_pose.pose.orientation),
      base_odom.twist.twist.linear.x,
      base_odom.twist.twist.linear.y,
      base_odom.twist.twist.angular.z,
      vx_samp, vy_samp, vtheta_samp);
  }

  RCLCPP_WARN(
    rclcpp::get_logger("TrajectoryPlannerROS"),
    "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
  return -1.0;
}

bool TrajectoryPlannerROS::isGoalReached()
{
  if (!isInitialized()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TrajectoryPlannerROS"),
      "This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  return reached_goal_;
}

}  // namespace base_local_planner