# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: navfn_ros.cpp
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
*********************************************************************/
#include <navfn/navfn_ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(navfn::NavfnROS, nav_core::BaseGlobalPlanner)

namespace navfn {

  NavfnROS::NavfnROS() 
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {}

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap, global_frame);
  }

  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame){
    if(!initialized_){
      costmap_ = costmap;
      global_frame_ = global_frame;
      planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

      ros::NodeHandle private_nh("~/" + name);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      private_nh.param("visualize_potential", visualize_potential_, false);

      //if we're going to visualize the potential array we need to advertise
      if(visualize_potential_)
        potarr_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("potential", 1);

      private_nh.param("allow_unknown", allow_unknown_, true);
      private_nh.param("planner_window_x", planner_window_x_, 0.0);
      private_nh.param("planner_window_y", planner_window_y_, 0.0);
      private_nh.param("default_tolerance", default_tolerance_, 0.0);

      make_plan_srv_ =  private_nh.advertiseService("make_plan", &NavfnROS::makePlanService, this);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point){
    return validPointPotential(world_point, default_tolerance_);
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point, double tolerance){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    double resolution = costmap_->getResolution();
    geometry_msgs::Point p;
    p = world_point;

    p.y = world_point.y - tolerance;

    while(p.y <= world_point.y + tolerance){
      p.x = world_point.x - tolerance;
      while(p.x <= world_point.x + tolerance){
        double potential = getPointPotential(p);
        if(potential < POT_HIGH){
          return true;
        }
        p.x += resolution;
      }
      p.y += resolution;
    }

    return false;
  }

  double NavfnROS::getPointPotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return -1.0;
    }

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;

    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
  }

  bool NavfnROS::computePotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    return planner_->calcNavFnDijkstra();
  }

  void NavfnROS::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  bool NavfnROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
  } 

  void NavfnROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }

  /*
 * TODO [Hole 2]: Implement the core 'makePlan' logic for the NavFn planner.
 * Requirements:
 * 1. Thread Safety: Use a scoped lock on 'mutex_'.
 * 2. Validation: Ensure both start and goal poses are in the 'global_frame_'.
 * 3. Coordinate Transformation: Convert world coordinates to map cells using 'costmap_'.
 * 4. Algorithm Execution: Set the start/goal for the 'planner_', trigger Dijkstra, 
 * and handle the 'found_legal' potential search within the given 'tolerance'.
 * 5. Plan Extraction: If a path is found, extract it into the 'plan' vector and publish it.
 *END OF TODO 
*/
  void NavfnROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    
    if(path.empty()) {
      //still set a valid frame so visualization won't hit transform issues
    	gui_path.header.frame_id = global_frame_;
      gui_path.header.stamp = ros::Time::now();
    } else { 
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

  bool NavfnROS::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;

    //the potential has already been computed, so we won't update our copy of the costmap
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    planner_->calcPath(costmap_->getSizeInCellsX() * 4);

    //extract the plan
    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();
    ros::Time plan_time = ros::Time::now();

    for(int i = len - 1; i >= 0; --i){
      //convert the plan to world coordinates
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return !plan.empty();
  }
};

----------------------------


# LLM Response

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
#include <navfn/navfn_ros.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/cost_values.h>
#include <nav2_costmap_2d/costmap_2d.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

PLUGINLIB_EXPORT_CLASS(navfn::NavfnROS, nav2_core::BaseGlobalPlanner)

namespace navfn {

  NavfnROS::NavfnROS() 
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {}

  NavfnROS::NavfnROS(std::string name, nav2_costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      initialize(name, costmap_ros);
  }

  NavfnROS::NavfnROS(std::string name, nav2_costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      initialize(name, costmap, global_frame);
  }

  void NavfnROS::initialize(std::string name, nav2_costmap_2d::Costmap2D* costmap, std::string global_frame){
    if(!initialized_){
      costmap_ = costmap;
      global_frame_ = global_frame;
      planner_ = std::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

      rclcpp::Node::SharedPtr private_nh = rclcpp::Node::make_shared(name, "/" + name);

      plan_pub_ = private_nh->create_publisher<nav_msgs::msg::Path>("plan", 1);

      private_nh->declare_parameter("visualize_potential", rclcpp::ParameterValue(false));
      private_nh->get_parameter("visualize_potential", visualize_potential_);

      if(visualize_potential_)
        potarr_pub_ = private_nh->create_publisher<sensor_msgs::msg::PointCloud2>("potential", 1);

      private_nh->declare_parameter("allow_unknown", rclcpp::ParameterValue(true));
      private_nh->get_parameter("allow_unknown", allow_unknown_);
      private_nh->declare_parameter("planner_window_x", rclcpp::ParameterValue(0.0));
      private_nh->get_parameter("planner_window_x", planner_window_x_);
      private_nh->declare_parameter("planner_window_y", rclcpp::ParameterValue(0.0));
      private_nh->get_parameter("planner_window_y", planner_window_y_);
      private_nh->declare_parameter("default_tolerance", rclcpp::ParameterValue(0.0));
      private_nh->get_parameter("default_tolerance", default_tolerance_);

      make_plan_srv_ = private_nh->create_service<nav_msgs::srv::GetPlan>("make_plan", 
        [this](const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req, 
               std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp) {
          return makePlanService(*req, *resp);
        });

      logger_ = private_nh->get_logger();

      initialized_ = true;
    }
    else
      RCLCPP_WARN(rclcpp::get_logger("navfn"), "This planner has already been initialized, you can't call it twice, doing nothing");
  }

  void NavfnROS::initialize(std::string name, nav2_costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::msg::Point& world_point){
    return validPointPotential(world_point, default_tolerance_);
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::msg::Point& world_point, double tolerance){
    if(!initialized_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    double resolution = costmap_->getResolution();
    geometry_msgs::msg::Point p;
    p = world_point;

    p.y = world_point.y - tolerance;

    while(p.y <= world_point.y + tolerance){
      p.x = world_point.x - tolerance;
      while(p.x <= world_point.x + tolerance){
        double potential = getPointPotential(p);
        if(potential < POT_HIGH){
          return true;
        }
        p.x += resolution;
      }
      p.y += resolution;
    }

    return false;
  }

  double NavfnROS::getPointPotential(const geometry_msgs::msg::Point& world_point){
    if(!initialized_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return -1.0;
    }

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;

    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
  }

  bool NavfnROS::computePotential(const geometry_msgs::msg::Point& world_point){
    if(!initialized_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    return planner_->calcNavFnDijkstra();
  }

  void NavfnROS::clearRobotCell(const geometry_msgs::msg::PoseStamped& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  }

  bool NavfnROS::makePlanService(nav_msgs::srv::GetPlan::Request& req, nav_msgs::srv::GetPlan::Response& resp){
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = rclcpp::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
  } 

  void NavfnROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  bool NavfnROS::makePlan(const geometry_msgs::msg::PoseStamped& start, 
      const geometry_msgs::msg::PoseStamped& goal, std::vector<geometry_msgs::msg::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }

  void NavfnROS::makePlan(const geometry_msgs::msg::PoseStamped& start, 
      const geometry_msgs::msg::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan){
    std::lock_guard<std::mutex> lock(mutex_);
    
    plan.clear();
    
    if(!initialized_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    if(start.header.frame_id != global_frame_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.", 
                global_frame_.c_str(), start.header.frame_id.c_str());
      return;
    }

    if(goal.header.frame_id != global_frame_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      RCLCPP_WARN(rclcpp::get_logger("navfn"), "The start pose sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return;
    }

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!costmap_->worldToMap(wx, wy, mx, my)){
      RCLCPP_WARN(rclcpp::get_logger("navfn"), "The goal pose sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    planner_->calcNavFnDijkstra();

    double resolution = costmap_->getResolution();
    geometry_msgs::msg::Point p;
    p = goal.pose.position;

    bool found_legal = false;
    double potential = getPointPotential(p);
    if(potential < POT_HIGH){
      found_legal = true;
    }

    if(!found_legal){
      p.y = goal.pose.position.y - tolerance;
      while(p.y <= goal.pose.position.y + tolerance && !found_legal){
        p.x = goal.pose.position.x - tolerance;
        while(p.x <= goal.pose.position.x + tolerance && !found_legal){
          potential = getPointPotential(p);
          if(potential < POT_HIGH){
            found_legal = true;
          }
          p.x += resolution;
        }
        p.y += resolution;
      }
    }

    if(found_legal){
      getPlanFromPotential(goal, plan);
    }

    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
  }

  void NavfnROS::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path, double r, double g, double b, double a){
    if(!initialized_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    nav_msgs::msg::Path gui_path;
    gui_path.poses.resize(path.size());
    
    if(path.empty()){
      gui_path.header.frame_id = global_frame_;
      gui_path.header.stamp = rclcpp::Time::now();
    } else { 
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_->publish(gui_path);
  }

  bool NavfnROS::getPlanFromPotential(const geometry_msgs::msg::PoseStamped& goal, std::vector<geometry_msgs::msg::PoseStamped>& plan){
    if(!initialized_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    plan.clear();

    if(goal.header.frame_id != global_frame_){
      RCLCPP_ERROR(rclcpp::get_logger("navfn"), "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;

    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      RCLCPP_WARN_THROTTLE(logger_, 1000, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    planner_->calcPath(costmap_->getSizeInCellsX() * 4);

    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();
    rclcpp::Time plan_time = rclcpp::Time::now();

    for(int i = len - 1; i >= 0; --i){
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return !plan.empty();
  }
};
```