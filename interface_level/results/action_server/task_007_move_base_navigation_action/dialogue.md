# Task: action_server/task_007_move_base_navigation_action

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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/msg/recovery_status.hpp>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(nullptr),
    planner_costmap_ros_(nullptr), controller_costmap_ros_(nullptr),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(nullptr), latest_plan_(nullptr), controller_plan_(nullptr),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_base", options);

    as_ = std::make_shared<MoveBaseActionServer>(
      node,
      "move_base",
      std::bind(&MoveBase::executeCb, this, std::placeholders::_1),
      false);

    rclcpp::Node::SharedPtr private_nh = node;
    rclcpp::Node::SharedPtr nh = node;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh->declare_parameter<std::string>("base_global_planner", "navfn/NavfnROS");
    private_nh->declare_parameter<std::string>("base_local_planner", "base_local_planner/TrajectoryPlannerROS");
    private_nh->declare_parameter<std::string>("global_costmap.robot_base_frame", "base_link");
    private_nh->declare_parameter<std::string>("global_costmap.global_frame", "map");
    private_nh->declare_parameter<double>("planner_frequency", 0.0);
    private_nh->declare_parameter<double>("controller_frequency", 20.0);
    private_nh->declare_parameter<double>("planner_patience", 5.0);
    private_nh->declare_parameter<double>("controller_patience", 15.0);
    private_nh->declare_parameter<int>("max_planning_retries", -1);

    private_nh->declare_parameter<double>("oscillation_timeout", 0.0);
    private_nh->declare_parameter<double>("oscillation_distance", 0.5);

    private_nh->declare_parameter<bool>("make_plan_clear_costmap", true);
    private_nh->declare_parameter<bool>("make_plan_add_unreachable_goal", true);

    private_nh->get_parameter("base_global_planner", global_planner);
    private_nh->get_parameter("base_local_planner", local_planner);
    private_nh->get_parameter("global_costmap.robot_base_frame", robot_base_frame_);
    private_nh->get_parameter("global_costmap.global_frame", global_frame_);
    private_nh->get_parameter("planner_frequency", planner_frequency_);
    private_nh->get_parameter("controller_frequency", controller_frequency_);
    private_nh->get_parameter("planner_patience", planner_patience_);
    private_nh->get_parameter("controller_patience", controller_patience_);
    private_nh->get_parameter("max_planning_retries", max_planning_retries_);

    private_nh->get_parameter("oscillation_timeout", oscillation_timeout_);
    private_nh->get_parameter("oscillation_distance", oscillation_distance_);

    private_nh->get_parameter("make_plan_clear_costmap", make_plan_clear_costmap_);
    private_nh->get_parameter("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_);

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(std::bind(&MoveBase::planThread, this));

    //for commanding the base
    vel_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh->create_publisher<geometry_msgs::msg::PoseStamped>("current_goal", rclcpp::QoS(1).transient_local());

    rclcpp::Node::SharedPtr action_nh = node;
    action_goal_pub_ = action_nh->create_publisher<move_base_msgs::msg::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_ = action_nh->create_publisher<move_base_msgs::msg::RecoveryStatus>("recovery_status", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    rclcpp::Node::SharedPtr simple_nh = node;
    goal_sub_ = simple_nh->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal", 1, std::bind(&MoveBase::goalCB, this, std::placeholders::_1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh->declare_parameter<double>("local_costmap.inscribed_radius", 0.325);
    private_nh->declare_parameter<double>("local_costmap.circumscribed_radius", 0.46);
    private_nh->declare_parameter<double>("clearing_radius", 0.46);
    private_nh->declare_parameter<double>("conservative_reset_dist", 3.0);

    private_nh->declare_parameter<bool>("shutdown_costmaps", false);
    private_nh->declare_parameter<bool>("clearing_rotation_allowed", true);
    private_nh->declare_parameter<bool>("recovery_behavior_enabled", true);

    private_nh->get_parameter("local_costmap.inscribed_radius", inscribed_radius_);
    private_nh->get_parameter("local_costmap.circumscribed_radius", circumscribed_radius_);
    private_nh->get_parameter("clearing_radius", clearing_radius_);
    private_nh->get_parameter("conservative_reset_dist", conservative_reset_dist_);

    private_nh->get_parameter("shutdown_costmaps", shutdown_costmaps_);
    private_nh->get_parameter("clearing_rotation_allowed", clearing_rotation_allowed_);
    private_nh->get_parameter("recovery_behavior_enabled", recovery_behavior_enabled_);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
      planner_ = bgp_loader_.createSharedInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(node->get_logger(), "Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      tc_ = blp_loader_.createSharedInstance(local_planner);
      RCLCPP_INFO(node->get_logger(), "Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_FATAL(node->get_logger(), "Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh->create_service<nav_msgs::srv::GetPlan>(
      "make_plan",
      std::bind(&MoveBase::planService, this, std::placeholders::_1, std::placeholders::_2));

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh->create_service<std_srvs::srv::Empty>(
      "clear_costmaps",
      std::bind(&MoveBase::clearCostmapsService, this, std::placeholders::_1, std::placeholders::_2));

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      RCLCPP_DEBUG(node->get_logger(), "Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(*private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    as_->start();

    // Dynamic reconfigure is not directly supported in ROS2, so this is omitted.
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    // Dynamic reconfigure is not directly supported in ROS2, so this is omitted.
  }

  void MoveBase::goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goal){
    RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::msg::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = rclcpp::Clock().now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_->publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::msg::PoseStamped global_pose;

    //clear the planner's costmap
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

    //clear the controller's costmap
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

  bool MoveBase::clearCostmapsService(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }


  bool MoveBase::planService(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
                             std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp){
    if(as_->is_active()){
      RCLCPP_ERROR(rclcpp::get_logger("move_base"), "move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == nullptr){
      RCLCPP_ERROR(rclcpp::get_logger("move_base"), "move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::msg::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req->start.header.frame_id.empty())
    {
        geometry_msgs::msg::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          RCLCPP_ERROR(rclcpp::get_logger("move_base"), "move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req->start;
    }

    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::msg::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req->goal, global_plan) || global_plan.empty()){
      RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req->goal.pose.position.x, req->goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::msg::PoseStamped p;
      p = req->goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req->tolerance > 0.0 && req->tolerance < search_increment) search_increment = req->tolerance;
      for(float max_offset = search_increment; max_offset <= req->tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req->goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req->goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req->goal);
                    }

                    found_legal = true;
                    RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp->plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp->plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != nullptr)
      as_.reset();

    if(planner_costmap_ros_ != nullptr)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != nullptr)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }

  bool MoveBase::makePlan(const geometry_msgs::msg::PoseStamped& goal, std::vector<geometry_msgs::msg::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("move_base"), "Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    geometry_msgs::msg::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      RCLCPP_WARN(rclcpp::get_logger("move_base"), "Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::msg::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
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
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      RCLCPP_ERROR(rclcpp::get_logger("move_base"), "Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      RCLCPP_ERROR(rclcpp::get_logger("move_base"), "Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      RCLCPP_ERROR(rclcpp::get_logger("move_base"), "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::msg::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::msg::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::msg::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = rclcpp::Time(0);

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      RCLCPP_WARN(rclcpp::get_logger("move_base"), "Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(){
    planner_cond_.notify_one();
  }

  void MoveBase::planThread(){
    RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Starting planner thread...");
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("move_base_plan_thread");
    rclcpp::TimerBase::SharedPtr timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(rclcpp::ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      rclcpp::Time start_time = rclcpp::Clock().now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::msg::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Planning...");

      //run planner
      planner_plan_->clear();
      bool gotPlan = rclcpp::ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){
        RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::msg::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = rclcpp::Clock().now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if(state_==PLANNING){
        RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "No Plan...");
        rclcpp::Time attempt_end = last_valid_plan_ + rclcpp::Duration::from_seconds(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&
           (rclcpp::Clock().now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        rclcpp::Duration sleep_time = (start_time + rclcpp::Duration::from_seconds(1.0/planner_frequency_)) - rclcpp::Clock().now();
        if (sleep_time > rclcpp::Duration(0,0)){
          wait_for_wake = true;
          timer = n->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_time.to_chrono<std::chrono::nanoseconds>()),
            std::bind(&MoveBase::wakePlanner, this));
        }
      }
    }
  }

  void MoveBase::executeCb(const std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> move_base_goal)
  {
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->abort(move_base_msgs::action::MoveBase::Result(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::msg::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    publishZeroVelocity();
    //we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_->publish(goal);

    rclcpp::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = rclcpp::Clock().now();
    last_valid_plan_ = rclcpp::Clock().now();
    last_oscillation_reset_ = rclcpp::Clock().now();
    planning_retries_ = 0;

    rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("move_base_execute_cb");
    while(rclcpp::ok())
    {
      if(c_freq_change_)
      {
        RCLCPP_INFO(rclcpp::get_logger("move_base"), "Setting controller frequency to %.2f", controller_frequency_);
        r = rclcpp::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if(as_->is_canceling()){
        if(as_->is_new_goal_available()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          auto new_goal_handle = as_->accept_new_goal();

          if(!isQuaternionValid(new_goal_handle->get_goal()->target_pose.pose.orientation)){
            as_->abort(move_base_msgs::action::MoveBase::Result(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal_handle->get_goal()->target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_->publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = rclcpp::Clock().now();
          last_valid_plan_ = rclcpp::Clock().now();
          last_oscillation_reset_ = rclcpp::Clock().now();
          planning_retries_ = 0;
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Move base preempting the current goal");
          as_->canceled();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_->publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = rclcpp::Clock().now();
        last_valid_plan_ = rclcpp::Clock().now();
        last_oscillation_reset_ = rclcpp::Clock().now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      rclcpp::Time start = rclcpp::Clock().now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(goal);

      //if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way

      rclcpp::Duration t_diff = rclcpp::Clock().now() - start;
      RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Full control cycle time: %.9f\n", t_diff.seconds());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycle_time() > rclcpp::Duration::from_seconds(1 / controller_frequency_) && state_ == CONTROLLING)
        RCLCPP_WARN(rclcpp::get_logger("move_base"), "Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycle_time().seconds());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    as_->abort(move_base_msgs::action::MoveBase::Result(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::executeCycle(geometry_msgs::msg::PoseStamped& goal){

    // ===== TODO: MoveBase Action Server Execution =====
    // Implement the main action server loop for handling navigation goals.
    // This should include:
    // - Receiving and validating new navigation goals
    // - Transforming goals to the global planning frame
    // - Starting the planner and enabling the control loop
    // - Publishing feedback about current robot position
    // - Executing velocity commands via the local planner
    // - Handling goal completion, preemption, or abortion
    // - Updating the state machine (PLANNING, CONTROLLING, CLEARING) accordingly
    // - Managing planner thread notifications and synchronization
    // The implementation should mimic the lifecycle of a ROS ActionServer for navigation.

    // Implementation:
    // Lock planner mutex for thread safety
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

    // Check if new global plan is available
    if (new_global_plan_) {
      new_global_plan_ = false;
      // Reset recovery index on new plan
      recovery_index_ = 0;
    }

    // Get current robot pose
    geometry_msgs::msg::PoseStamped current_pose;
    if (!getRobotPose(current_pose, controller_costmap_ros_)) {
      RCLCPP_WARN(rclcpp::get_logger("move_base"), "Failed to get robot pose");
      publishZeroVelocity();
      return false;
    }

    // Publish feedback (current robot pose)
    move_base_msgs::action::MoveBase::Feedback feedback_msg;
    feedback_msg.base_position = current_pose;
    as_->publish_feedback(feedback_msg);

    switch(state_) {
      case PLANNING:
        // Wait for planner to produce a plan
        if (new_global_plan_) {
          state_ = CONTROLLING;
          last_valid_plan_ = rclcpp::Clock().now();
          last_valid_control_ = rclcpp::Clock().now();
          planning_retries_ = 0;
        } else {
          // If planner not running, wake it up
          if (!runPlanner_) {
            runPlanner_ = true;
            planner_cond_.notify_one();
          }
        }
        publishZeroVelocity();
        break;

      case CONTROLLING:
        if (latest_plan_->empty()) {
          RCLCPP_WARN(rclcpp::get_logger("move_base"), "No valid plan available");
          state_ = PLANNING;
          break;
        }

        // Check if goal reached
        if (distance(current_pose, goal) <= inscribed_radius_) {
          publishZeroVelocity();
          as_->succeed(move_base_msgs::action::MoveBase::Result());
          resetState();
          return true;
        }

        // Check oscillation timeout and distance
        if ((rclcpp::Clock().now() - last_oscillation_reset_).seconds() > oscillation_timeout_) {
          if (distance(current_pose, last_oscillation_pose_) < oscillation_distance_) {
            RCLCPP_WARN(rclcpp::get_logger("move_base"), "Oscillation detected");
            state_ = CLEARING;
            recovery_trigger_ = OSCILLATION_R;
            break;
          }
          last_oscillation_pose_ = current_pose;
          last_oscillation_reset_ = rclcpp::Clock().now();
        }

        // Run local planner to compute velocity commands
        geometry_msgs::msg::Twist cmd_vel;
        if (!tc_->computeVelocityCommands(cmd_vel)) {
          RCLCPP_WARN(rclcpp::get_logger("move_base"), "Local planner failed to produce a valid command");
          state_ = CLEARING;
          recovery_trigger_ = CONTROLLING_R;
          break;
        }

        // Publish velocity command
        vel_pub_->publish(cmd_vel);
        last_valid_control_ = rclcpp::Clock().now();
        break;

      case CLEARING:
        publishZeroVelocity();

        if (recovery_index_ >= recovery_behaviors_.size()) {
          RCLCPP_ERROR(rclcpp::get_logger("move_base"), "All recovery behaviors have failed. Aborting.");
          as_->abort(move_base_msgs::action::MoveBase::Result(), "All recovery behaviors failed");
          resetState();
          return true;
        }

        // Execute recovery behavior
        RCLCPP_INFO(rclcpp::get_logger("move_base"), "Executing recovery behavior %s", recovery_behavior_names_[recovery_index_].c_str());
        recovery_behaviors_[recovery_index_]->runBehavior();

        recovery_index_++;
        state_ = PLANNING;
        runPlanner_ = true;
        planner_cond_.notify_one();
        break;

      default:
        RCLCPP_ERROR(rclcpp::get_logger("move_base"), "Unknown state in MoveBase");
        publishZeroVelocity();
        as_->abort(move_base_msgs::action::MoveBase::Result(), "Unknown state");
        resetState();
        return true;
    }

    return false;
    //END OF TODO
  }

  bool MoveBase::loadRecoveryBehaviors(rclcpp::Node& node){
    // XmlRpc is not supported in ROS2 in the same way; this function would require reimplementation or parameter parsing.
    // For migration, assume failure to load user recovery behaviors.
    return false;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("~");
      n->set_parameter(rclcpp::Parameter("conservative_reset.reset_distance", conservative_reset_dist_));
      n->set_parameter(rclcpp::Parameter("aggressive_reset.reset_distance", circumscribed_radius_ * 4));

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createSharedInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createSharedInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createSharedInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_){
        recovery_behaviors_.push_back(rotate);
        recovery_behavior_names_.push_back("rotate_recovery");
      }
    }
    catch(pluginlib::PluginlibException& ex){
      RCLCPP_FATAL(rclcpp::get_logger("move_base"), "Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      RCLCPP_DEBUG(rclcpp::get_logger("move_base"), "Stopping costmaps");
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
    robot_pose.header.stamp = rclcpp::Time(0); // latest available
    rclcpp::Time current_time = rclcpp::Clock().now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("move_base"), *costmap->getName().c_str(), 1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("move_base"), *costmap->getName().c_str(), 1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("move_base"), *costmap->getName().c_str(), 1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (!global_pose.header.stamp.is_zero() &&
        current_time.seconds() - global_pose.header.stamp.seconds() > costmap->getTransformTolerance())
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("move_base"), 1.0, "Transform timeout for %s. Current time: %.4f, pose stamp: %.4f, tolerance: %.4f",
                        costmap->getName().c_str(),
                        current_time.seconds(), global_pose.header.stamp.seconds(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
```