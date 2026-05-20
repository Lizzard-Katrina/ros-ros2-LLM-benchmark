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

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/msg/path.hpp>

#include <tf2/utils.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace base_local_planner {

  void TrajectoryPlannerROS::reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        //Avoid looping
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }
      tc_->reconfigure(config);
      reached_goal_ = false;
  }

  TrajectoryPlannerROS::TrajectoryPlannerROS() :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {}

  TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }

  void TrajectoryPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if (! isInitialized()) {

      node_ = rclcpp::Node::make_shared(name);
      auto logger = node_->get_logger();
      
      g_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("global_plan", 1);
      l_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double sim_time, sim_granularity, angular_sim_granularity;
      int vx_samples, vtheta_samples;
      double path_distance_bias, goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
      bool holonomic_robot, dwa, simple_attractor, heading_scoring;
      double heading_scoring_timestep;
      double max_vel_x, min_vel_x;
      double backup_vel;
      double stop_time_buffer;
      std::string world_model_type;
      rotating_to_goal_ = false;

      //initialize the copy of the costmap the controller will use
      costmap_ = costmap_ros_->getCostmap();

      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      
      node_->declare_parameter("prune_plan", true);
      node_->get_parameter("prune_plan", prune_plan_);

      node_->declare_parameter("yaw_goal_tolerance", 0.05);
      node_->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
      
      node_->declare_parameter("xy_goal_tolerance", 0.10);
      node_->get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
      
      node_->declare_parameter("acc_lim_x", 2.5);
      node_->get_parameter("acc_lim_x", acc_lim_x_);
      
      node_->declare_parameter("acc_lim_y", 2.5);
      node_->get_parameter("acc_lim_y", acc_lim_y_);
      
      node_->declare_parameter("acc_lim_theta", 3.2);
      node_->get_parameter("acc_lim_theta", acc_lim_theta_);

      node_->declare_parameter("stop_time_buffer", 0.2);
      node_->get_parameter("stop_time_buffer", stop_time_buffer);

      node_->declare_parameter("latch_xy_goal_tolerance", false);
      node_->get_parameter("latch_xy_goal_tolerance", latch_xy_goal_tolerance_);

      if(node_->has_parameter("acc_limit_x"))
        RCLCPP_ERROR(logger, "You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(node_->has_parameter("acc_limit_y"))
        RCLCPP_ERROR(logger, "You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(node_->has_parameter("acc_limit_th"))
        RCLCPP_ERROR(logger, "You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      double controller_frequency = 20.0;
      node_->declare_parameter("controller_frequency", 20.0);
      node_->get_parameter("controller_frequency", controller_frequency);
      if(controller_frequency > 0)
        sim_period_ = 1.0 / controller_frequency;
      else
      {
        RCLCPP_WARN(logger, "A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
      RCLCPP_INFO(logger, "Sim period is set to %.2f", sim_period_);

      node_->declare_parameter("sim_time", 1.0);
      node_->get_parameter("sim_time", sim_time);
      
      node_->declare_parameter("sim_granularity", 0.025);
      node_->get_parameter("sim_granularity", sim_granularity);
      
      node_->declare_parameter("angular_sim_granularity", sim_granularity);
      node_->get_parameter("angular_sim_granularity", angular_sim_granularity);
      
      node_->declare_parameter("vx_samples", 3);
      node_->get_parameter("vx_samples", vx_samples);
      
      node_->declare_parameter("vtheta_samples", 20);
      node_->get_parameter("vtheta_samples", vtheta_samples);

      node_->declare_parameter("path_distance_bias", 0.6);
      node_->get_parameter("path_distance_bias", path_distance_bias);
      
      node_->declare_parameter("goal_distance_bias", 0.6);
      node_->get_parameter("goal_distance_bias", goal_distance_bias);

      node_->declare_parameter("occdist_scale", 0.01);
      node_->get_parameter("occdist_scale", occdist_scale);

      bool meter_scoring = false;
      node_->declare_parameter("meter_scoring", false);
      if (!node_->has_parameter("meter_scoring")) {
        RCLCPP_WARN(logger, "Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.");
      } else {
        node_->get_parameter("meter_scoring", meter_scoring);

        if(meter_scoring) {
          double resolution = costmap_->getResolution();
          goal_distance_bias *= resolution;
          path_distance_bias *= resolution;
        } else {
          RCLCPP_WARN(logger, "Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settings robust against changes of costmap resolution.");
        }
      }

      node_->declare_parameter("heading_lookahead", 0.325);
      node_->get_parameter("heading_lookahead", heading_lookahead);
      
      node_->declare_parameter("oscillation_reset_dist", 0.05);
      node_->get_parameter("oscillation_reset_dist", oscillation_reset_dist);
      
      node_->declare_parameter("escape_reset_dist", 0.10);
      node_->get_parameter("escape_reset_dist", escape_reset_dist);
      
      node_->declare_parameter("escape_reset_theta", M_PI_4);
      node_->get_parameter("escape_reset_theta", escape_reset_theta);
      
      node_->declare_parameter("holonomic_robot", true);
      node_->get_parameter("holonomic_robot", holonomic_robot);
      
      node_->declare_parameter("max_vel_x", 0.5);
      node_->get_parameter("max_vel_x", max_vel_x);
      
      node_->declare_parameter("min_vel_x", 0.1);
      node_->get_parameter("min_vel_x", min_vel_x);

      double max_rotational_vel;
      node_->declare_parameter("max_rotational_vel", 1.0);
      node_->get_parameter("max_rotational_vel", max_rotational_vel);
      max_vel_th_ = max_rotational_vel;
      min_vel_th_ = -1.0 * max_rotational_vel;

      node_->declare_parameter("min_in_place_vel_theta", 0.4);
      node_->get_parameter("min_in_place_vel_theta", min_in_place_vel_th_);
      
      reached_goal_ = false;
      backup_vel = -0.1;
      
      node_->declare_parameter("escape_vel", backup_vel);
      node_->get_parameter("escape_vel", backup_vel);

      if(backup_vel >= 0.0)
        RCLCPP_WARN(logger, "You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

      node_->declare_parameter("world_model", std::string("costmap"));
      node_->get_parameter("world_model", world_model_type);
      
      node_->declare_parameter("dwa", true);
      node_->get_parameter("dwa", dwa);
      
      node_->declare_parameter("heading_scoring", false);
      node_->get_parameter("heading_scoring", heading_scoring);
      
      node_->declare_parameter("heading_scoring_timestep", 0.8);
      node_->get_parameter("heading_scoring_timestep", heading_scoring_timestep);

      simple_attractor = false;

      double min_pt_separation, max_obstacle_height, grid_resolution;
      node_->declare_parameter("point_grid.max_sensor_range", 2.0);
      node_->get_parameter("point_grid.max_sensor_range", max_sensor_range_);
      
      node_->declare_parameter("point_grid.min_pt_separation", 0.01);
      node_->get_parameter("point_grid.min_pt_separation", min_pt_separation);
      
      node_->declare_parameter("point_grid.max_obstacle_height", 2.0);
      node_->get_parameter("point_grid.max_obstacle_height", max_obstacle_height);
      
      node_->declare_parameter("point_grid.grid_resolution", 0.2);
      node_->get_parameter("point_grid.grid_resolution", grid_resolution);

      if(world_model_type != "costmap") {
        RCLCPP_ERROR(logger, "At this time, only costmap world models are supported by this controller");
      }
      
      world_model_ = new CostmapModel(*costmap_);
      std::vector<double> y_vels = loadYVels(node_);

      footprint_spec_ = costmap_ros_->getRobotFootprint();

      tc_ = new TrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
          acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, path_distance_bias,
          goal_distance_bias, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

      map_viz_.initialize(name,
                          global_frame_,
                          [this](int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost){
                              return tc_->getCellCosts(cx, cy, path_cost, goal_cost, occ_cost, total_cost);
                          });
      initialized_ = true;

    } else {
      RCLCPP_WARN(rclcpp::get_logger("trajectory_planner_ros"), "This planner has already been initialized, doing nothing");
    }
  }

  std::vector<double> TrajectoryPlannerROS::loadYVels(rclcpp::Node::SharedPtr node){
    std::vector<double> y_vels;

    std::string y_vel_list;
    node->declare_parameter("y_vels", std::string(""));
    if(node->get_parameter("y_vels", y_vel_list) && !y_vel_list.empty()){
      typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
      boost::char_separator<char> sep("[], ");
      tokenizer tokens(y_vel_list, sep);

      for(tokenizer::iterator i = tokens.begin(); i != tokens.end(); i++){
        y_vels.push_back(atof((*i).c_str()));
      }
    }
    else{
      //if no values are passed in, we'll provide defaults
      y_vels.push_back(-0.3);
      y_vels.push_back(-0.1);
      y_vels.push_back(0.1);
      y_vels.push_back(0.3);
    }

    return y_vels;
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS() {
    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;
  }

  bool TrajectoryPlannerROS::stopWithAccLimits(const geometry_msgs::msg::PoseStamped& global_pose, const geometry_msgs::msg::PoseStamped& robot_vel, geometry_msgs::msg::Twist& cmd_vel){
    double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim_x_ * sim_period_));
    double vy = sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim_y_ * sim_period_));

    double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

    double yaw = tf2::getYaw(global_pose.pose.orientation);
    bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
        robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, vx, vy, vth);

    if(valid_cmd){
      RCLCPP_DEBUG(node_->get_logger(), "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
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

  bool TrajectoryPlannerROS::rotateToGoal(const geometry_msgs::msg::PoseStamped& global_pose, const geometry_msgs::msg::PoseStamped& robot_vel, double goal_th, geometry_msgs::msg::Twist& cmd_vel){
    double yaw = tf2::getYaw(global_pose.pose.orientation);
    double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
        std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
        std::min(-1.0 * min_in_place_vel_th_, ang_diff));

    double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
    double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    v_theta_samp = v_theta_samp > 0.0
      ? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp ))
      : std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));

    bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
        robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, 0.0, 0.0, v_theta_samp);

    RCLCPP_DEBUG(node_->get_logger(), "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd){
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;

  }

  bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan){
    if (! isInitialized()) {
      RCLCPP_ERROR(rclcpp::get_logger("trajectory_planner_ros"), "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    global_plan_.clear();
    global_plan_ = orig_global_plan;
    
    xy_tolerance_latch_ = false;
    reached_goal_ = false;
    return true;
  }

  bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::msg::Twist& cmd_vel){
    if (! isInitialized()) {
      RCLCPP_ERROR(node_->get_logger(), "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::msg::PoseStamped> local_plan;
    geometry_msgs::msg::PoseStamped global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
      return false;
    }

    std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
    if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform the global plan to the frame of the controller");
      return false;
    }

    if(prune_plan_)
      prunePlan(global_pose, transformed_plan, global_plan_);

    geometry_msgs::msg::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = robot_base_frame_;

    geometry_msgs::msg::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    if(transformed_plan.empty())
      return false;

    const geometry_msgs::msg::PoseStamped& goal_point = transformed_plan.back();
    const double goal_x = goal_point.pose.position.x;
    const double goal_y = goal_point.pose.position.y;

    const double yaw = tf2::getYaw(goal_point.pose.orientation);

    double goal_th = yaw;

    if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

      if (latch_xy_goal_tolerance_) {
        xy_tolerance_latch_ = true;
      }

      double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
      if (fabs(angle) <= yaw_goal_tolerance_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        reached_goal_ = true;
      } else {
        tc_->updatePlan(transformed_plan);
        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        map_viz_.publishCostCloud(costmap_);

        nav_msgs::msg::Odometry base_odom;
        odom_helper_.getOdom(base_odom);

        if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
          if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
            return false;
          }
        }
        else{
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
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
      RCLCPP_DEBUG(node_->get_logger(),
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);
      return false;
    }

    RCLCPP_DEBUG(node_->get_logger(), "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = node_->now();
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

  bool TrajectoryPlannerROS::checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
       geometry_msgs::msg::PoseStamped global_pose;
       if(costmap_ros_->getRobotPose(global_pose)){
         if(update_map){
           std::vector<geometry_msgs::msg::PoseStamped> plan;
           plan.push_back(global_pose);
           tc_->updatePlan(plan, true);
         }
         geometry_msgs::msg::PoseStamped robot_vel;
         odom_helper_.getRobotVel(robot_vel);
         return tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation),
             robot_vel.pose.position.x,
             robot_vel.pose.position.y,
             tf2::getYaw(robot_vel.pose.orientation), vx_samp, vy_samp, vtheta_samp);
       }
       RCLCPP_WARN(node_->get_logger(), "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
       return false;
  }


  double TrajectoryPlannerROS::scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    geometry_msgs::msg::PoseStamped global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        plan.push_back(global_pose);
        tc_->updatePlan(plan, true);
      }

      nav_msgs::msg::Odometry base_odom;
      {
        std::lock_guard<std::mutex> lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->scoreTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

    }
    RCLCPP_WARN(node_->get_logger(), "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return -1.0;
  }

  bool TrajectoryPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      RCLCPP_ERROR(node_->get_logger(), "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    return reached_goal_; 
  }
};