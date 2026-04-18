# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
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
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", [this](auto& goal){ executeCb(goal); }, false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(std::bind(&MoveBase::planThread, this));

    //for commanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, [this](auto& goal){ goalCB(goal); });

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    as_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
    dsrv_->setCallback(cb);
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

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

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }


  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req.start;
    }

    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
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

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
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

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
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
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    publishZeroVelocity();
    //we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_.publish(goal);

    ros::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if(as_->isPreemptRequested()){
        if(as_->isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal.target_pose);

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
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

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
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(goal);

      //if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal){

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
    //END OF TODO
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
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
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
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
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (!global_pose.header.stamp.isZero() &&
        current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};

----------------------------


# LLM Response

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
#include <move_base_msgs/action/recovery_status.hpp>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <geometry_msgs/msg/twist.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace move_base {

  MoveBase::MoveBase(const rclcpp::NodeOptions & options) :
    nav2_util::LifecycleNode("move_base", "", options),
    tf_(std::shared_ptr<rclcpp::Clock>(this->get_clock())),
    bgp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
    blp_loader_("nav2_core", "nav2_core::LocalPlanner"),
    recovery_loader_("nav2_core", "nav2_core::Recovery"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

    RCLCPP_INFO(get_logger(), "Creating move_base");

    // Declare parameters
    declare_parameter("base_global_planner", rclcpp::ParameterValue(std::string("nav2_navfn_planner/NavfnPlanner")));
    declare_parameter("base_local_planner", rclcpp::ParameterValue(std::string("nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController")));
    declare_parameter("global_costmap.robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    declare_parameter("global_costmap.global_frame", rclcpp::ParameterValue(std::string("map")));
    declare_parameter("planner_frequency", rclcpp::ParameterValue(0.0));
    declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
    declare_parameter("planner_patience", rclcpp::ParameterValue(5.0));
    declare_parameter("controller_patience", rclcpp::ParameterValue(15.0));
    declare_parameter("max_planning_retries", rclcpp::ParameterValue(-1));
    declare_parameter("oscillation_timeout", rclcpp::ParameterValue(0.0));
    declare_parameter("oscillation_distance", rclcpp::ParameterValue(0.5));
    declare_parameter("make_plan_clear_costmap", rclcpp::ParameterValue(true));
    declare_parameter("make_plan_add_unreachable_goal", rclcpp::ParameterValue(true));
    declare_parameter("local_costmap.inscribed_radius", rclcpp::ParameterValue(0.325));
    declare_parameter("local_costmap.circumscribed_radius", rclcpp::ParameterValue(0.46));
    declare_parameter("clearing_radius", rclcpp::ParameterValue(0.46));
    declare_parameter("conservative_reset_dist", rclcpp::ParameterValue(3.0));
    declare_parameter("shutdown_costmaps", rclcpp::ParameterValue(false));
    declare_parameter("clearing_rotation_allowed", rclcpp::ParameterValue(true));
    declare_parameter("recovery_behavior_enabled", rclcpp::ParameterValue(true));

    recovery_trigger_ = PLANNING_R;

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::msg::PoseStamped>();
  }

  nav2_util::CallbackReturn
  MoveBase::on_configure(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Configuring move_base");

    // Get parameters
    get_parameter("base_global_planner", global_planner_);
    get_parameter("base_local_planner", local_planner_);
    get_parameter("global_costmap.robot_base_frame", robot_base_frame_);
    get_parameter("global_costmap.global_frame", global_frame_);
    get_parameter("planner_frequency", planner_frequency_);
    get_parameter("controller_frequency", controller_frequency_);
    get_parameter("planner_patience", planner_patience_);
    get_parameter("controller_patience", controller_patience_);
    get_parameter("max_planning_retries", max_planning_retries_);
    get_parameter("oscillation_timeout", oscillation_timeout_);
    get_parameter("oscillation_distance", oscillation_distance_);
    get_parameter("make_plan_clear_costmap", make_plan_clear_costmap_);
    get_parameter("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_);
    get_parameter("local_costmap.inscribed_radius", inscribed_radius_);
    get_parameter("local_costmap.circumscribed_radius", circumscribed_radius_);
    get_parameter("clearing_radius", clearing_radius_);
    get_parameter("conservative_reset_dist", conservative_reset_dist_);
    get_parameter("shutdown_costmaps", shutdown_costmaps_);
    get_parameter("clearing_rotation_allowed", clearing_rotation_allowed_);
    get_parameter("recovery_behavior_enabled", recovery_behavior_enabled_);

    // Create costmaps
    planner_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{get_namespace()}, tf_);
    planner_costmap_ros_->configure();
    planner_costmap_ros_->pause();

    controller_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap", std::string{get_namespace()}, tf_);
    controller_costmap_ros_->configure();
    controller_costmap_ros_->pause();

    // Initialize global planner
    try {
      planner_ = bgp_loader_.createUniqueInstance(global_planner_);
      planner_->configure(planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create global planner %s: %s", 
                   global_planner_.c_str(), ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    // Initialize local planner
    try {
      tc_ = blp_loader_.createUniqueInstance(local_planner_);
      RCLCPP_INFO(get_logger(), "Created local planner %s", local_planner_.c_str());
      tc_->configure(controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create local planner %s: %s", 
                   local_planner_.c_str(), ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    // Create action server
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "move_base",
      std::bind(&MoveBase::executeCb, this, std::placeholders::_1),
      nullptr,
      std::chrono::milliseconds(500),
      true);

    // Create publishers
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    current_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("current_goal", 1);
    action_goal_pub_ = create_publisher<move_base_msgs::msg::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_ = create_publisher<move_base_msgs::msg::RecoveryStatus>("recovery_status", 1);

    // Create subscribers
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal", 1, std::bind(&MoveBase::goalCB, this, std::placeholders::_1));

    // Create services
    make_plan_srv_ = create_service<nav_msgs::srv::GetPlan>(
      "make_plan", std::bind(&MoveBase::planService, this, 
      std::placeholders::_1, std::placeholders::_2));
    
    clear_costmaps_srv_ = create_service<std_srvs::srv::Empty>(
      "clear_costmaps", std::bind(&MoveBase::clearCostmapsService, this,
      std::placeholders::_1, std::placeholders::_2));

    // Start planner thread
    planner_thread_ = std::make_unique<std::thread>(std::bind(&MoveBase::planThread, this));

    // Load recovery behaviors
    if (!loadRecoveryBehaviors()) {
      loadDefaultRecoveryBehaviors();
    }

    // Initial state
    state_ = PLANNING;
    recovery_index_ = 0;

    // Start costmaps
    planner_costmap_ros_->activate();
    controller_costmap_ros_->activate();

    if (shutdown_costmaps_) {
      RCLCPP_DEBUG(get_logger(), "Stopping costmaps initially");
      planner_costmap_ros_->deactivate();
      controller_costmap_ros_->deactivate();
    }

    // Setup parameter callback
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    param_sub_ = param_client_->on_parameter_event(
      std::bind(&MoveBase::reconfigureCB, this, std::placeholders::_1));

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  MoveBase::on_activate(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Activating move_base");
    
    vel_pub_->on_activate();
    current_goal_pub_->on_activate();
    action_goal_pub_->on_activate();
    recovery_status_pub_->on_activate();
    
    action_server_->activate();
    
    if (!shutdown_costmaps_) {
      planner_costmap_ros_->activate();
      controller_costmap_ros_->activate();
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  MoveBase::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Deactivating move_base");
    
    action_server_->deactivate();
    resetState();
    
    vel_pub_->on_deactivate();
    current_goal_pub_->on_deactivate();
    action_goal_pub_->on_deactivate();
    recovery_status_pub_->on_deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  MoveBase::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up move_base");

    planner_thread_->join();
    planner_thread_.reset();

    planner_.reset();
    tc_.reset();

    planner_costmap_ros_->cleanup();
    controller_costmap_ros_->cleanup();

    planner_costmap_ros_.reset();
    controller_costmap_ros_.reset();

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_plan_ = NULL;
    latest_plan_ = NULL;
    controller_plan_ = NULL;

    action_server_.reset();
    vel_pub_.reset();
    current_goal_pub_.reset();
    action_goal_pub_.reset();
    recovery_status_pub_.reset();
    goal_sub_.reset();
    make_plan_srv_.reset();
    clear_costmaps_srv_.reset();

    recovery_behaviors_.clear();
    recovery_behavior_names_.clear();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  MoveBase::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "Shutting down move_base");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void MoveBase::reconfigureCB(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    std::lock_guard<std::recursive_mutex> lock(configuration_mutex_);

    if (!setup_) {
      setup_ = true;
      return;
    }

    for (const auto & changed_parameter : event->changed_parameters) {
      const std::string & name = changed_parameter.name;
      const rclcpp::Parameter & param = get_parameter(name);

      if (name == "planner_frequency") {
        planner_frequency_ = param.as_double();
        p_freq_change_ = true;
      } else if (name == "controller_frequency") {
        controller_frequency_ = param.as_double();
        c_freq_change_ = true;
      } else if (name == "planner_patience") {
        planner_patience_ = param.as_double();
      } else if (name == "controller_patience") {
        controller_patience_ = param.as_double();
      } else if (name == "max_planning_retries") {
        max_planning_retries_ = param.as_int();
      } else if (name == "conservative_reset_dist") {
        conservative_reset_dist_ = param.as_double();
      } else if (name == "recovery_behavior_enabled") {
        recovery_behavior_enabled_ = param.as_bool();
      } else if (name == "clearing_rotation_allowed") {
        clearing_rotation_allowed_ = param.as_bool();
      } else if (name == "shutdown_costmaps") {
        shutdown_costmaps_ = param.as_bool();
      } else if (name == "oscillation_timeout") {
        oscillation_timeout_ = param.as_double();
      } else if (name == "oscillation_distance") {
        oscillation_distance_ = param.as_double();
      } else if (name == "base_global_planner") {
        std::string new_global_planner = param.as_string();
        if (new_global_planner != global_planner_) {
          try {
            std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
            planner_plan_->clear();
            latest_plan_->clear();
            controller_plan_->clear();
            resetState();
            
            planner_ = bgp_loader_.createUniqueInstance(new_global_planner);
            planner_->configure(planner_costmap_ros_);
            global_planner_ = new_global_planner;
            RCLCPP_INFO(get_logger(), "Loaded global planner %s", global_planner_.c_str());
          } catch (const pluginlib::PluginlibException & ex) {
            RCLCPP_FATAL(get_logger(), "Failed to create global planner %s: %s", 
                         new_global_planner.c_str(), ex.what());
            set_parameter(rclcpp::Parameter("base_global_planner", global_planner_));
          }
        }
      } else if (name == "base_local_planner") {
        std::string new_local_planner = param.as_string();
        if (new_local_planner != local_planner_) {
          try {
            planner_plan_->clear();
            latest_plan_->clear();
            controller_plan_->clear();
            resetState();
            
            tc_ = blp_loader_.createUniqueInstance(new_local_planner);
            tc_->configure(controller_costmap_ros_);
            local_planner_ = new_local_planner;
            RCLCPP_INFO(get_logger(), "Loaded local planner %s", local_planner_.c_str());
          } catch (const pluginlib::PluginlibException & ex) {
            RCLCPP_FATAL(get_logger(), "Failed to create local planner %s: %s", 
                         new_local_planner.c_str(), ex.what());
            set_parameter(rclcpp::Parameter("base_local_planner", local_planner_));
          }
        }
      } else if (name == "make_plan_clear_costmap") {
        make_plan_clear_costmap_ = param.as_bool();
      } else if (name == "make_plan_add_unreachable_goal") {
        make_plan_add_unreachable_goal_ = param.as_bool();
      }
    }
  }

  void MoveBase::goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr goal){
    RCLCPP_DEBUG(get_logger(), "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::msg::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = now();
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

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, nav2_costmap_2d::FREE_SPACE);

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

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, nav2_costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*resp*/){
    
    //clear the costmaps
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }


  bool MoveBase::planService(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp){
    
    if(action_server_->is_active()){
      RCLCPP_ERROR(get_logger(), "move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      RCLCPP_ERROR(get_logger(), "move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::msg::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req->start.header.frame_id.empty())
    {
        geometry_msgs::msg::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          RCLCPP_ERROR(get_logger(), "move_base cannot make a plan for you because it could not get the start pose of the robot");
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
    if(!planner_->createPlan(start, req->goal, global_plan) || global_plan.empty()){
      RCLCPP_DEBUG(get_logger(), "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
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

                if(planner_->createPlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req->goal);
                    }

                    found_legal = true;
                    RCLCPP_DEBUG(get_logger(), "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  RCLCPP_DEBUG(get_logger(), "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
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

    if(planner_thread_ && planner_thread_->joinable()) {
      planner_thread_->join();
    }

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;
  }

  bool MoveBase::makePlan(const geometry_msgs::msg::PoseStamped& goal, std::vector<geometry_msgs::msg::PoseStamped>& plan){
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      RCLCPP_ERROR(get_logger(), "Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    geometry_msgs::msg::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      RCLCPP_WARN(get_logger(), "Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::msg::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->createPlan(start, goal, plan) || plan.empty()){
      RCLCPP_DEBUG(get_logger(), "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
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
      RCLCPP_ERROR(get_logger(), "Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      RCLCPP_ERROR(get_logger(), "Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      RCLCPP_ERROR(get_logger(), "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
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
      RCLCPP_WARN(get_logger(), "Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(){
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  void MoveBase::planThread(){
    RCLCPP_DEBUG(get_logger(), "Starting planner thread...");
    std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
    while(rclcpp::ok()){
      //check if we should run the planner (the mutex is locked)
      while(!runPlanner_){
        //if we should not be running the planner then suspend this thread
        RCLCPP_DEBUG(get_logger(), "Planner thread is suspending");
        planner_cond_.wait(lock);
      }
      rclcpp::Time start_time = now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::msg::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      RCLCPP_DEBUG(get_logger(), "Planning...");

      //run planner
      planner_plan_->clear();
      bool gotPlan = rclcpp::ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){
        RCLCPP_DEBUG(get_logger(), "Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::msg::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        RCLCPP_DEBUG(get_logger(), "Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if(state_==PLANNING){
        RCLCPP_DEBUG(get_logger(), "No Plan...");
        rclcpp::Time attempt_end = last_valid_plan_ + rclcpp::Duration::from_seconds(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&
           (now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
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
        rclcpp::Duration sleep_time = (start_time + rclcpp::Duration::from_seconds(1.0/planner_frequency_)) - now();
        if (sleep_time.seconds() > 0.0){
          auto timer = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_time.to_chrono<std::chrono::nanoseconds>()),
            std::bind(&MoveBase::wakePlanner, this));
          planner_cond_.wait(lock);
          timer->cancel();
        }
      }
    }
  }

  void MoveBase::executeCb(const std::shared_ptr<GoalHandleMoveBase> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    
    if(!isQuaternionValid(goal->target_pose.pose.orientation)){
      auto result = std::make_shared<move_base_msgs::action::MoveBase::Result>();
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::msg::PoseStamped goal_pose = goalToGlobalFrame(goal->target_pose);

    publishZeroVelocity();
    //we have a goal so start the planner
    std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal_pose;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_->publish(goal_pose);

    rclcpp::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      RCLCPP_DEBUG(get_logger(), "Starting up costmaps that were shut down previously");
      planner_costmap_ros_->activate();
      controller_costmap_ros_->activate();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = now();
    last_valid_plan_ = now();
    last_oscillation_reset_ = now();
    planning_retries_ = 0;

    auto feedback = std::make_shared<move_base_msgs::action::MoveBase::Feedback>();
    auto result = std::make_shared<move_base_msgs::action::MoveBase::Result>();

    while(rclcpp::ok())
    {
      if(c_freq_change_)
      {
        RCLCPP_INFO(get_logger(), "Setting controller frequency to %.2f", controller_frequency_);
        r = rclcpp::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if(goal_handle->is_canceling()){
        if(goal_handle->is_canceling()){
          resetState();
          goal_handle->canceled(result);
          RCLCPP_DEBUG(get_logger(), "Move base preempting the current goal");
          return;
        }
      }

      // Check for new goal
      if(!goal_handle->is_executing()){
        auto new_goal = goal_handle->get_goal();
        if(new_goal){
          if(!isQuaternionValid(new_goal->target_pose.pose.orientation)){
            goal_handle->abort(result);
            RCLCPP_WARN(get_logger(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal_pose = goalToGlobalFrame(new_goal->target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal_pose;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          RCLCPP_DEBUG(get_logger(), "move_base has received a goal of x: %.2f, y: %.2f", 
                      goal_pose.pose.position.x, goal_pose.pose.position.y);
          current_goal_pub_->publish(goal_pose);

          //make sure to reset our timeouts and counters
          last_valid_control_ = now();
          last_valid_plan_ = now();
          last_oscillation_reset_ = now();
          planning_retries_ = 0;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal_pose.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal_pose = goalToGlobalFrame(goal_pose);

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal_pose;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        RCLCPP_DEBUG(get_logger(), "The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", 
                    goal_pose.header.frame_id.c_str(), goal_pose.pose.position.x, goal_pose.pose.position.y);
        current_goal_pub_->publish(goal_pose);

        //make sure to reset our timeouts and counters
        last_valid_control_ = now();
        last_valid_plan_ = now();
        last_oscillation_reset_ = now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      auto start = std::chrono::steady_clock::now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(goal_pose, feedback, goal_handle);

      //if we're done, then we'll return from execute
      if(done) {
        goal_handle->succeed(result);
        return;
      }

      //check if execution of the goal has completed in some way
      auto end = std::chrono::steady_clock::now();
      std::chrono::duration<double> t_diff = end - start;
      RCLCPP_DEBUG(get_logger(), "Full control cycle time: %.9f\n", t_diff.count());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.period() > rclcpp::Duration::from_seconds(1 / controller_frequency_) && state_ == CONTROLLING)
        RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", 
                   controller_frequency_, r.period().seconds());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::executeCycle(
    geometry_msgs::msg::PoseStamped& goal,
    std::shared_ptr<move_base_msgs::action::MoveBase::Feedback> feedback,
    const std::shared_ptr<GoalHandleMoveBase> goal_handle){
    
    // Update feedback with current pose
    geometry_msgs::msg::PoseStamped global_pose;
    if(getRobotPose(global_pose, planner_costmap_ros_)) {
      feedback->base_position = global_pose;
      goal_handle->publish_feedback(feedback);
    }

    // Check for oscillation
    if(oscillation_timeout_ > 0.0 && last_oscillation_reset_ + rclcpp::Duration::from_seconds(oscillation_timeout_) < now()) {
      publishZeroVelocity();
      state_ = CLEARING;
      recovery_trigger_ = OSCILLATION_R;
    }

    // State machine
    switch(state_) {
      case PLANNING:
        {
          // Wait for plan
          std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
          if(new_global_plan_) {
            new_global_plan_ = false;
            if(!latest_plan_->empty()) {
              *controller_plan_ = *latest_plan_;
              state_ = CONTROLLING;
              RCLCPP_DEBUG(get_logger(), "Got new plan, switching to CONTROLLING state");
            }
          }
        }
        break;

      case CONTROLLING:
        {
          // Check if goal reached
          if(tc_->isGoalReached()) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            resetState();
            return true;
          }

          // Check for valid control
          if(last_valid_control_ + rclcpp::Duration::from_seconds(controller_patience_) < now()) {
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
            RCLCPP_WARN(get_logger(), "Aborting because controller failed for too long");
            break;
          }

          // Check for oscillation
          if(oscillation_timeout_ > 0.0) {
            geometry_msgs::msg::PoseStamped pose;
            if(getRobotPose(pose, controller_costmap_ros_)) {
              if(distance(pose, oscillation_pose_) >= oscillation_distance_) {
                last_oscillation_reset_ = now();
                oscillation_pose_ = pose;
              }
            }
          }

          // Get velocity command
          geometry_msgs::msg::Twist cmd_vel;
          bool ok = tc_->computeVelocityCommands(cmd_vel);
          
          if(ok) {
            last_valid_control_ = now();
            vel_pub_->publish(cmd_vel);
          } else {
            RCLCPP_WARN(get_logger(), "Local planner failed to compute velocity command");
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
        }
        break;

      case CLEARING:
        {
          if(!recovery_behavior_enabled_) {
            RCLCPP_WARN(get_logger(), "Recovery behaviors disabled, aborting");
            resetState();
            return false;
          }

          // Execute recovery behavior
          if(recovery_index_ < recovery_behaviors_.size()) {
            RCLCPP_DEBUG(get_logger(), "Executing recovery behavior %s", 
                        recovery_behavior_names_[recovery_index_].c_str());
            
            move_base_msgs::msg::RecoveryStatus status;
            status.pose_stamped = goal;
            status.current_recovery_number = recovery_index_;
            status.total_number_of_recoveries = recovery_behaviors_.size();
            status.recovery_behavior_name = recovery_behavior_names_[recovery_index_];
            recovery_status_pub_->publish(status);

            recovery_behaviors_[recovery_index_]->runBehavior();
            recovery_index_++;
          } else {
            RCLCPP_WARN(get_logger(), "All recovery behaviors failed, aborting");
            resetState();
            return false;
          }

          // After recovery, go back to planning
          state_ = PLANNING;
          recovery_index_ = 0;

          // Wake planner
          std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        break;
    }

    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(){
    auto node = shared_from_this();
    if(!node->has_parameter("recovery_behaviors")) {
      return false;
    }

    rclcpp::Parameter recovery_behaviors_param = node->get_parameter("recovery_behaviors");
    if(recovery_behaviors_param.get_type() != rclcpp::ParameterType::PARAMETER_ARRAY) {
      RCLCPP_ERROR(get_logger(), "Recovery behaviors must be specified as an array");
      return false;
    }

    auto behavior_list = recovery_behaviors_param.as_string_array();
    for(size_t i = 0; i < behavior_list.size(); ++i) {
      // Parse behavior string (format: "type:name")
      std::vector<std::string> parts;
      boost::split(parts, behavior_list[i], boost::is_any_of(":"));
      
      if(parts.size() != 2) {
        RCLCPP_ERROR(get_logger(), "Recovery behavior must be in format 'type:name'");
        return false;
      }

      std::string type = parts[0];
      std::string name = parts[1];

      // Check for duplicate names
      for(size_t j = i + 1; j < behavior_list.size(); ++j) {
        std::vector<std::string> other_parts;
        boost::split(other_parts, behavior_list[j], boost::is_any_of(":"));
        if(other_parts.size() == 2 && other_parts[1] == name) {
          RCLCPP_ERROR(get_logger(), "A recovery behavior with the name %s already exists", name.c_str());
          return false;
        }
      }

      try {
        auto behavior = recovery_loader_.createUniqueInstance(type);
        behavior->configure(planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back(name);
        recovery_behaviors_.push_back(std::move(behavior));
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(get_logger(), "Failed to load recovery behavior %s: %s", type.c_str(), ex.what());
        return false;
      }
    }

    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    recovery_behavior_names_.clear();
    
    try {
      // Conservative clear costmap
      auto cons_clear = recovery_loader_.createUniqueInstance("clear_costmap_recovery/ClearCostmapRecovery");
      cons_clear->configure(planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(std::move(cons_clear));

      // Rotate recovery
      if(clearing_rotation_allowed_){
        auto rotate = recovery_loader_.createUniqueInstance("rotate_recovery/RotateRecovery");
        rotate->configure(planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(std::move(rotate));
      }

      // Aggressive clear costmap
      auto ags_clear = recovery_loader_.createUniqueInstance("clear_costmap_recovery/ClearCostmapRecovery");
      ags_clear->configure(planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(std::move(ags_clear));

      // Rotate again
      if(clearing_rotation_allowed_){
        auto rotate2 = recovery_loader_.createUniqueInstance("rotate_recovery/RotateRecovery");
        rotate2->configure(planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(std::move(rotate2));
      }
    }
    catch(pluginlib::PluginlibException& ex){
      RCLCPP_FATAL(get_logger(), "Failed to load default recovery behaviors: %s", ex.what());
    }
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    std::unique_lock<std::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      RCLCPP_DEBUG(get_logger(), "Stopping costmaps");
      planner_costmap_ros_->deactivate();
      controller_costmap_ros_->deactivate();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::msg::PoseStamped& global_pose, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::msg::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = rclcpp::Time(0); // latest available
    rclcpp::Time current_time = now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (!global_pose.header.stamp.seconds() == 0 &&
        current_time.seconds() - global_pose.header.stamp.seconds() > costmap->getTransformTolerance())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.seconds(), global_pose.header.stamp.seconds(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};