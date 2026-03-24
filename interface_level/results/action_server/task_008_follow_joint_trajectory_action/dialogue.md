# Task: action_server/task_008_follow_joint_trajectory_action

```cpp
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

#include <nav_core/parameter_magic.h>
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
      world_model_(nullptr), tc_(nullptr), costmap_ros_(nullptr), tf_(nullptr), setup_(false), initialized_(false), odom_helper_("odom") {}

  TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) :
      world_model_(nullptr), tc_(nullptr), costmap_ros_(nullptr), tf_(nullptr), setup_(false), initialized_(false), odom_helper_("odom") {

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }

  void TrajectoryPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if (! isInitialized()) {

      rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(name);
      rclcpp::Node::SharedPtr private_nh = node->get_node_base_interface()->get_node();

      g_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("global_plan", 1);
      l_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

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
      node->declare_parameter("prune_plan", true);
      prune_plan_ = node->get_parameter("prune_plan").as_bool();

      node->declare_parameter("yaw_goal_tolerance", 0.05);
      yaw_goal_tolerance_ = node->get_parameter("yaw_goal_tolerance").as_double();
      node->declare_parameter("xy_goal_tolerance", 0.10);
      xy_goal_tolerance_ = node->get_parameter("xy_goal_tolerance").as_double();
      node->declare_parameter("acc_lim_x", 2.5);
      acc_lim_x_ = node->get_parameter("acc_lim_x").as_double();
      node->declare_parameter("acc_lim_y", 2.5);
      acc_lim_y_ = node->get_parameter("acc_lim_y").as_double();
      node->declare_parameter("acc_lim_theta", 3.2);
      acc_lim_theta_ = node->get_parameter("acc_lim_theta").as_double();

      node->declare_parameter("stop_time_buffer", 0.2);
      stop_time_buffer = node->get_parameter("stop_time_buffer").as_double();

      node->declare_parameter("latch_xy_goal_tolerance", false);
      latch_xy_goal_tolerance_ = node->get_parameter("latch_xy_goal_tolerance").as_bool();

      // Since ROS2 parameters are strict, we do not check for deprecated params here.

      std::string controller_frequency_param_name = "controller_frequency";
      double controller_frequency = 20.0;
      if (node->has_parameter(controller_frequency_param_name)) {
        controller_frequency = node->get_parameter(controller_frequency_param_name).as_double();
      }
      if(controller_frequency > 0)
        sim_period_ = 1.0 / controller_frequency;
      else
      {
        RCLCPP_WARN(node->get_logger(), "A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
      RCLCPP_INFO(node->get_logger(), "Sim period is set to %.2f", sim_period_);

      node->declare_parameter("sim_time", 1.0);
      sim_time = node->get_parameter("sim_time").as_double();
      node->declare_parameter("sim_granularity", 0.025);
      sim_granularity = node->get_parameter("sim_granularity").as_double();
      node->declare_parameter("angular_sim_granularity", sim_granularity);
      angular_sim_granularity = node->get_parameter("angular_sim_granularity").as_double();
      node->declare_parameter("vx_samples", 3);
      vx_samples = node->get_parameter("vx_samples").as_int();
      node->declare_parameter("vtheta_samples", 20);
      vtheta_samples = node->get_parameter("vtheta_samples").as_int();

      path_distance_bias = nav_core::loadParameterWithDeprecation(*node,
                                                                  "path_distance_bias",
                                                                  "pdist_scale",
                                                                  0.6);
      goal_distance_bias = nav_core::loadParameterWithDeprecation(*node,
                                                                  "goal_distance_bias",
                                                                  "gdist_scale",
                                                                  0.6);
      // values of the deprecated params need to be applied to the current params, as defaults 
      // of defined for dynamic reconfigure will override them otherwise.
      if (node->has_parameter("pdist_scale") && !node->has_parameter("path_distance_bias"))
      {
        node->set_parameter(rclcpp::Parameter("path_distance_bias", path_distance_bias));
      }
      if (node->has_parameter("gdist_scale") && !node->has_parameter("goal_distance_bias"))
      {
        node->set_parameter(rclcpp::Parameter("goal_distance_bias", goal_distance_bias));
      }

      node->declare_parameter("occdist_scale", 0.01);
      occdist_scale = node->get_parameter("occdist_scale").as_double();

      bool meter_scoring = false;
      if (!node->has_parameter("meter_scoring")) {
        RCLCPP_WARN(node->get_logger(), "Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.");
      } else {
        meter_scoring = node->get_parameter("meter_scoring").as_bool();

        if(meter_scoring) {
          //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
          double resolution = costmap_->getResolution();
          goal_distance_bias *= resolution;
          path_distance_bias *= resolution;
        } else {
          RCLCPP_WARN(node->get_logger(), "Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settings robust against changes of costmap resolution.");
        }
      }

      node->declare_parameter("heading_lookahead", 0.325);
      heading_lookahead = node->get_parameter("heading_lookahead").as_double();
      node->declare_parameter("oscillation_reset_dist", 0.05);
      oscillation_reset_dist = node->get_parameter("oscillation_reset_dist").as_double();
      node->declare_parameter("escape_reset_dist", 0.10);
      escape_reset_dist = node->get_parameter("escape_reset_dist").as_double();
      node->declare_parameter("escape_reset_theta", M_PI_4);
      escape_reset_theta = node->get_parameter("escape_reset_theta").as_double();
      node->declare_parameter("holonomic_robot", true);
      holonomic_robot = node->get_parameter("holonomic_robot").as_bool();
      node->declare_parameter("max_vel_x", 0.5);
      max_vel_x = node->get_parameter("max_vel_x").as_double();
      node->declare_parameter("min_vel_x", 0.1);
      min_vel_x = node->get_parameter("min_vel_x").as_double();

      double max_rotational_vel = 1.0;
      node->declare_parameter("max_rotational_vel", max_rotational_vel);
      max_rotational_vel = node->get_parameter("max_rotational_vel").as_double();
      max_vel_th_ = max_rotational_vel;
      min_vel_th_ = -1.0 * max_rotational_vel;

      min_in_place_vel_th_ = nav_core::loadParameterWithDeprecation(*node,
                                                                    "min_in_place_vel_theta",
                                                                    "min_in_place_rotational_vel", 0.4);
      reached_goal_ = false;
      backup_vel = -0.1;
      if(node->has_parameter("backup_vel"))
      {
        backup_vel = node->get_parameter("backup_vel").as_double();
        RCLCPP_WARN(node->get_logger(), "The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");
      }

      //if both backup_vel and escape_vel are set... we'll use escape_vel
      if(node->has_parameter("escape_vel"))
      {
        backup_vel = node->get_parameter("escape_vel").as_double();
      }

      if(backup_vel >= 0.0)
        RCLCPP_WARN(node->get_logger(), "You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

      node->declare_parameter("world_model", std::string("costmap"));
      world_model_type = node->get_parameter("world_model").as_string();
      node->declare_parameter("dwa", true);
      dwa = node->get_parameter("dwa").as_bool();
      node->declare_parameter("heading_scoring", false);
      heading_scoring = node->get_parameter("heading_scoring").as_bool();
      node->declare_parameter("heading_scoring_timestep", 0.8);
      heading_scoring_timestep = node->get_parameter("heading_scoring_timestep").as_double();

      simple_attractor = false;

      //parameters for using the freespace controller
      double min_pt_separation, max_obstacle_height, grid_resolution;
      node->declare_parameter("point_grid/max_sensor_range", 2.0);
      max_sensor_range_ = node->get_parameter("point_grid/max_sensor_range").as_double();
      node->declare_parameter("point_grid/min_pt_separation", 0.01);
      min_pt_separation = node->get_parameter("point_grid/min_pt_separation").as_double();
      node->declare_parameter("point_grid/max_obstacle_height", 2.0);
      max_obstacle_height = node->get_parameter("point_grid/max_obstacle_height").as_double();
      node->declare_parameter("point_grid/grid_resolution", 0.2);
      grid_resolution = node->get_parameter("point_grid/grid_resolution").as_double();

      RCLCPP_ASSERT(node->get_logger(), world_model_type == "costmap" && "At this time, only costmap world models are supported by this controller");
      world_model_ = new CostmapModel(*costmap_);
      std::vector<double> y_vels = loadYVels(*node);

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

      // dynamic_reconfigure is not available in ROS2, so this is omitted

    } else {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPlannerROS"), "This planner has already been initialized, doing nothing");
    }
  }

  std::vector<double> TrajectoryPlannerROS::loadYVels(rclcpp::Node& node){
    std::vector<double> y_vels;

    std::string y_vel_list;
    if(node.get_parameter("y_vels", y_vel_list)){
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
    //make sure to clean things up
    delete tc_;
    delete world_model_;
  }

  bool TrajectoryPlannerROS::stopWithAccLimits(const geometry_msgs::msg::PoseStamped& global_pose, const geometry_msgs::msg::PoseStamped& robot_vel, geometry_msgs::msg::Twist& cmd_vel){
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim_x_ * sim_period_));
    double vy = sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim_y_ * sim_period_));

    double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

    //we do want to check whether or not the command is valid
    double yaw = tf2::getYaw(global_pose.pose.orientation);
    bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
        robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, vx, vy, vth);

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd){
      RCLCPP_DEBUG(rclcpp::get_logger("TrajectoryPlannerROS"), "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
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

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
    double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
    v_theta_samp = v_theta_samp > 0.0
      ? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp ))
      : std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
        robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw, 0.0, 0.0, v_theta_samp);

    RCLCPP_DEBUG(rclcpp::get_logger("TrajectoryPlannerROS"), "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd){
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;

  }

  bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& orig_global_plan){
    if (! isInitialized()) {
      RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlannerROS"), "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    //reset the at goal flag
    reached_goal_ = false;
    return true;
  }

  bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::msg::Twist& cmd_vel){
    if (! isInitialized()) {
      RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlannerROS"), "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::msg::PoseStamped> local_plan;
    geometry_msgs::msg::PoseStamped global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
      return false;
    }

    std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPlannerROS"), "Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      prunePlan(global_pose, transformed_plan, global_plan_);

    geometry_msgs::msg::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = robot_base_frame_;

    geometry_msgs::msg::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;

    const geometry_msgs::msg::PoseStamped& goal_point = transformed_plan.back();
    //we assume the global goal is the last point in the global plan
    const double goal_x = goal_point.pose.position.x;
    const double goal_y = goal_point.pose.position.y;

    const double yaw = tf2::getYaw(goal_point.pose.orientation);

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {

      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if (latch_xy_goal_tolerance_) {
        xy_tolerance_latch_ = true;
      }

      double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
      //check to see if the goal orientation has been reached
      if (fabs(angle) <= yaw_goal_tolerance_) {
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        reached_goal_ = true;
      } else {
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids
        tc_->updatePlan(transformed_plan);
        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        map_viz_.publishCostCloud(costmap_);

        //copy over the odometry information
        nav_msgs::msg::Odometry base_odom;
        odom_helper_.getOdom(base_odom);

        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
          if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
            return false;
          }
        }
        //if we're stopped... then we want to rotate to goal
        else{
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
            return false;
          }
        }
      }

      //publish an empty plan because we've reached our goal position
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    tc_->updatePlan(transformed_plan);

    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

    map_viz_.publishCostCloud(costmap_);
    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    RCLCPP_INFO(rclcpp::get_logger("TrajectoryPlannerROS"), "Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    if (path.cost_ < 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("trajectory_planner_ros"),
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);
      return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("trajectory_planner_ros"), "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = rclcpp::Clock().now();
      pose.pose.position.x = p_x;
      pose.pose.position.y = p_y;
      pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, pose.pose.orientation);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    publishPlan(transformed_plan, g_plan_pub_);
    publishPlan(local_plan, l_plan_pub_);
    return true;
  }

  bool TrajectoryPlannerROS::checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
       // Translate this trajectory validity check to ROS2.
       geometry_msgs::msg::PoseStamped global_pose;
       if(costmap_ros_->getRobotPose(global_pose)){
         if(update_map){
           //we need to give the planner some sort of global plan, since we're only checking for legality
           //we'll just give the robot's current position
           std::vector<geometry_msgs::msg::PoseStamped> plan;
           plan.push_back(global_pose);
           tc_->updatePlan(plan, true);
         }

         //copy over the odometry information
         nav_msgs::msg::Odometry base_odom;
         {
           std::lock_guard<std::mutex> lock(odom_lock_);
           base_odom = base_odom_;
         }

         return tc_->checkTrajectory(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation),
             base_odom.twist.twist.linear.x,
             base_odom.twist.twist.linear.y,
             base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

       }
       RCLCPP_WARN(rclcpp::get_logger("TrajectoryPlannerROS"), "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
       return false;
  }


  double TrajectoryPlannerROS::scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    // Copy of checkTrajectory that returns a score instead of True / False
    geometry_msgs::msg::PoseStamped global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        plan.push_back(global_pose);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
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
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryPlannerROS"), "Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return -1.0;
  }

  bool TrajectoryPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlannerROS"), "This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //return flag set in controller
    return reached_goal_; 
  }
};
```