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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/msg/recovery_status.hpp>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf, rclcpp::Node::SharedPtr node) :
    tf_(tf),
    node_(node),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

    as_ = rclcpp_action::create_server<move_base_msgs::action::MoveBase>(
      node_,
      "move_base",
      std::bind(&MoveBase::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveBase::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveBase::handle_accepted, this, std::placeholders::_1));

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    node_->declare_parameter("base_global_planner", std::string("navfn/NavfnROS"));
    node_->declare_parameter("base_local_planner", std::string("base_local_planner/TrajectoryPlannerROS"));
    node_->declare_parameter("global_costmap.robot_base_frame", std::string("base_link"));
    node_->declare_parameter("global_costmap.global_frame", std::string("map"));
    node_->declare_parameter("planner_frequency", 0.0);
    node_->declare_parameter("controller_frequency", 20.0);
    node_->declare_parameter("planner_patience", 5.0);
    node_->declare_parameter("controller_patience", 15.0);
    node_->declare_parameter("max_planning_retries", -1);

    node_->declare_parameter("oscillation_timeout", 0.0);
    node_->declare_parameter("oscillation_distance", 0.5);

    node_->declare_parameter("make_plan_clear_costmap", true);
    node_->declare_parameter("make_plan_add_unreachable_goal", true);

    std::string global_planner, local_planner;
    node_->get_parameter("base_global_planner", global_planner);
    node_->get_parameter("base_local_planner", local_planner);
    node_->get_parameter("global_costmap.robot_base_frame", robot_base_frame_);
    node_->get_parameter("global_costmap.global_frame", global_frame_);
    node_->get_parameter("planner_frequency", planner_frequency_);
    node_->get_parameter("controller_frequency", controller_frequency_);
    node_->get_parameter("planner_patience", planner_patience_);
    node_->get_parameter("controller_patience", controller_patience_);
    node_->get_parameter("max_planning_retries", max_planning_retries_);

    node_->get_parameter("oscillation_timeout", oscillation_timeout_);
    node_->get_parameter("oscillation_distance", oscillation_distance_);
    node_->get_parameter("make_plan_clear_costmap", make_plan_clear_costmap_);
    node_->get_parameter("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_);

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new std::thread(std::bind(&MoveBase::planThread, this));

    //for commanding the base
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    current_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("current_goal", 0 );

    action_goal_pub_ = node_->create_publisher<move_base_msgs::msg::MoveBaseActionGoal>("move_base/goal", 1);
    recovery_status_pub_= node_->create_publisher<move_base_msgs::msg::RecoveryStatus>("move_base/recovery_status", 1);

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "move_base_simple/goal", 1, std::bind(&MoveBase::goalCB, this, std::placeholders::_1));

    node_->declare_parameter("local_costmap.inscribed_radius", 0.325);
    node_->declare_parameter("local_costmap.circumscribed_radius", 0.46);
    node_->declare_parameter("clearing_radius", 0.46);
    node_->declare_parameter("conservative_reset_dist", 3.0);
    node_->declare_parameter("shutdown_costmaps", false);
    node_->declare_parameter("clearing_rotation_allowed", true);
    node_->declare_parameter("recovery_behavior_enabled", true);

    node_->get_parameter("local_costmap.inscribed_radius", inscribed_radius_);
    node_->get_parameter("local_costmap.circumscribed_radius", circumscribed_radius_);
    node_->get_parameter("clearing_radius", clearing_radius_);
    node_->get_parameter("conservative_reset_dist", conservative_reset_dist_);
    node_->get_parameter("shutdown_costmaps", shutdown_costmaps_);
    node_->get_parameter("clearing_rotation_allowed", clearing_rotation_allowed_);
    node_->get_parameter("recovery_behavior_enabled", recovery_behavior_enabled_);

    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_, node_);
    planner_costmap_ros_->pause();

    try {
      planner_ = bgp_loader_.createSharedInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(node_->get_logger(), "Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_, node_);
    controller_costmap_ros_->pause();

    try {
      tc_ = blp_loader_.createSharedInstance(local_planner);
      RCLCPP_INFO(node_->get_logger(), "Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(node_->get_logger(), "Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    make_plan_srv_ = node_->create_service<nav_msgs::srv::GetPlan>("make_plan", std::bind(&MoveBase::planService, this, std::placeholders::_1, std::placeholders::_2));
    clear_costmaps_srv_ = node_->create_service<std_srvs::srv::Empty>("clear_costmaps", std::bind(&MoveBase::clearCostmapsService, this, std::placeholders::_1, std::placeholders::_2));

    if(shutdown_costmaps_){
      RCLCPP_DEBUG(node_->get_logger(), "Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    if(!loadRecoveryBehaviors(node_)){
      loadDefaultRecoveryBehaviors();
    }

    state_ = PLANNING;
    recovery_index_ = 0;
  }

  void MoveBase::goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goal){
    RCLCPP_DEBUG(node_->get_logger(), "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::msg::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = node_->now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_->publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::msg::PoseStamped global_pose;

    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::msg::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::msg::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> resp){
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }


  bool MoveBase::planService(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req, std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp){
    if(planner_costmap_ros_ == NULL){
      RCLCPP_ERROR(node_->get_logger(), "move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::msg::PoseStamped start;
    if(req->start.header.frame_id.empty())
    {
        geometry_msgs::msg::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          RCLCPP_ERROR(node_->get_logger(), "move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req->start;
    }

    if (make_plan_clear_costmap_) {
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    std::vector<geometry_msgs::msg::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req->goal, global_plan) || global_plan.empty()){
      RCLCPP_DEBUG(node_->get_logger(), "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req->goal.pose.position.x, req->goal.pose.position.y);

      geometry_msgs::msg::PoseStamped p;
      p = req->goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req->tolerance > 0.0 && req->tolerance < search_increment) search_increment = req->tolerance;
      for(float max_offset = search_increment; max_offset <= req->tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req->goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req->goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      global_plan.push_back(req->goal);
                    }

                    found_legal = true;
                    RCLCPP_DEBUG(node_->get_logger(), "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  RCLCPP_DEBUG(node_->get_logger(), "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    resp->plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp->plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    runPlanner_ = false;
    planner_cond_.notify_one();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }

  bool MoveBase::makePlan(const geometry_msgs::msg::PoseStamped& goal, std::vector<geometry_msgs::msg::PoseStamped>& plan){
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    plan.clear();

    if(planner_costmap_ros_ == NULL) {
      RCLCPP_ERROR(node_->get_logger(), "Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    geometry_msgs::msg::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      RCLCPP_WARN(node_->get_logger(), "Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::msg::PoseStamped& start = global_pose;

    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      RCLCPP_DEBUG(node_->get_logger(), "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_->publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::msg::Quaternion& q){
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      RCLCPP_ERROR(node_->get_logger(), "Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    if(tf_q.length2() < 1e-6){
      RCLCPP_ERROR(node_->get_logger(), "Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      RCLCPP_ERROR(node_->get_logger(), "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::msg::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::msg::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::msg::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    goal_pose.header.stamp = rclcpp::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      RCLCPP_WARN(node_->get_logger(), "Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner()
  {
    planner_cond_.notify_one();
  }

  void MoveBase::planThread(){
    RCLCPP_DEBUG(node_->get_logger(), "Starting planner thread...");
    rclcpp::TimerBase::SharedPtr timer;
    bool wait_for_wake = false;
    std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
    while(rclcpp::ok()){
      while(wait_for_wake || !runPlanner_){
        RCLCPP_DEBUG(node_->get_logger(), "Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      rclcpp::Time start_time = node_->now();

      geometry_msgs::msg::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      RCLCPP_DEBUG(node_->get_logger(), "Planning...");

      planner_plan_->clear();
      bool gotPlan = rclcpp::ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){
        RCLCPP_DEBUG(node_->get_logger(), "Got Plan with %zu points!", planner_plan_->size());
        std::vector<geometry_msgs::msg::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = node_->now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        RCLCPP_DEBUG(node_->get_logger(), "Generated a plan from the base_global_planner");

        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      else if(state_==PLANNING){
        RCLCPP_DEBUG(node_->get_logger(), "No Plan...");
        rclcpp::Time attempt_end = last_valid_plan_ + rclcpp::Duration::from_seconds(planner_patience_);

        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&
           (node_->now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          state_ = CLEARING;
          runPlanner_ = false;
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      lock.lock();

      if(planner_frequency_ > 0){
        rclcpp::Duration sleep_time = (start_time + rclcpp::Duration::from_seconds(1.0/planner_frequency_)) - node_->now();
        if (sleep_time > rclcpp::Duration::from_seconds(0.0)){
          wait_for_wake = true;
          timer = node_->create_wall_timer(std::chrono::duration<double>(sleep_time.seconds()), std::bind(&MoveBase::wakePlanner, this));
        }
      }
    }
  }

  double MoveBase::distance(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::executeCycle(geometry_msgs::msg::PoseStamped& goal){
    // ===== TODO: MoveBase Action Server Execution =====
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::PoseStamped global_pose;
    
    getRobotPose(global_pose, controller_costmap_ros_);

    if(tc_->isGoalReached()){
      publishZeroVelocity();
      return true;
    }

    if(state_ == CONTROLLING){
      if(tc_->computeVelocityCommands(cmd_vel)){
        last_valid_control_ = node_->now();
        vel_pub_->publish(cmd_vel);
      } else {
        publishZeroVelocity();
        state_ = CLEARING;
        recovery_trigger_ = CONTROLLING_R;
      }
    } else if(state_ == CLEARING){
      if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
        recovery_behaviors_[recovery_index_]->runBehavior();
        recovery_index_++;
        state_ = PLANNING;
      } else {
        publishZeroVelocity();
        return true; // Abort
      }
    }
    //END OF TODO
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(rclcpp::Node::SharedPtr node){
    return false;
  }

  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    return;
  }

  void MoveBase::resetState(){
    std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    if(shutdown_costmaps_){
      RCLCPP_DEBUG(node_->get_logger(), "Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::msg::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::msg::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = rclcpp::Time(); 
    rclcpp::Time current_time = node_->now();  

    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    if (global_pose.header.stamp.nanoseconds() != 0 &&
        current_time.seconds() - rclcpp::Time(global_pose.header.stamp).seconds() > costmap->getTransformTolerance())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.seconds(), rclcpp::Time(global_pose.header.stamp).seconds(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};