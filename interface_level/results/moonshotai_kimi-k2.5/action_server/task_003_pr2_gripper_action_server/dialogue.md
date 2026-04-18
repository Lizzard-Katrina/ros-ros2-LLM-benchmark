# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
private:
  typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
public:
  JointTrajectoryExecuter(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "joint_trajectory_action",
                   boost::bind(&JointTrajectoryExecuter::goalCB, this, _1),
                   boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1),
                   false),
    has_active_goal_(false)
  {
    using namespace XmlRpc;
    ros::NodeHandle pn("~");

    // Gets all of the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!pn.getParam("joints", joint_names))
    {
      ROS_FATAL("No joints given. (namespace: %s)", pn.getNamespace().c_str());
      exit(1);
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_FATAL("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
      exit(1);
    }
    for (int i = 0; i < joint_names.size(); ++i)
    {
      XmlRpcValue &name_value = joint_names[i];
      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)",
                  pn.getNamespace().c_str());
        exit(1);
      }

      joint_names_.push_back((std::string)name_value);
    }

    pn.param("constraints/goal_time", goal_time_constraint_, 0.0);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns = std::string("constraints/") + joint_names_[i];
      double g, t;
      pn.param(ns + "/goal", g, -1.0);
      pn.param(ns + "/trajectory", t, -1.0);
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }
    pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);


    pub_controller_command_ =
      node_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    sub_controller_state_ =
      node_.subscribe("state", 1, &JointTrajectoryExecuter::controllerStateCB, this);

    watchdog_timer_ = node_.createTimer(ros::Duration(1.0), &JointTrajectoryExecuter::watchdog, this);

    ros::Time started_waiting_for_controller = ros::Time::now();
    while (ros::ok() && !last_controller_state_)
    {
      ros::spinOnce();
      if (started_waiting_for_controller != ros::Time(0) &&
          ros::Time::now() > started_waiting_for_controller + ros::Duration(30.0))
      {
        ROS_WARN("Waited for the controller for 30 seconds, but it never showed up.");
        started_waiting_for_controller = ros::Time(0);
      }
      ros::Duration(0.1).sleep();
    }

    action_server_.start();
  }

  ~JointTrajectoryExecuter()
  {
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.stop();
  }

private:

  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size() != b.size())
      return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
      if (count(b.begin(), b.end(), a[i]) != 1)
        return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
      if (count(a.begin(), a.end(), b[i]) != 1)
        return false;
    }

    return true;
  }

  void watchdog(const ros::TimerEvent &e)
  {
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                 (now - last_controller_state_->header.stamp).toSec());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_.publish(empty);

        // Marks the current goal as aborted.
        active_goal_.setAborted();
        has_active_goal_ = false;
      }
    }
  }

  void goalCB(GoalHandle gh)
  {
   // TODO
   //Fill in the logic to handle incoming action goals:
   //- Accept a new goal from the client.
   //- Cancel any currently active goal if present.
   //- Publish the trajectory contained in the goal to the controller.
   //- Properly mark the new goal as accepted.
   // END of TODO

  }

  void cancelCB(GoalHandle gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }


  ros::NodeHandle node_;
  JTAS action_server_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Timer watchdog_timer_;

  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_traj_;


  std::vector<std::string> joint_names_;
  std::map<std::string,double> goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr last_controller_state_;
  void controllerStateCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)
  {
   //TODO:
   //Fill in the logic to monitor controller feedback and update goal state:
   //- Track the execution of the currently active trajectory.
   //- Check if the controller maintains the trajectory within constraints.
   //- If constraints are violated, abort the active goal.
   //- If the trajectory reaches the goal within allowed tolerances, mark the goal as succeeded.
   // END OF TODO

  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_trajectory_action_node");
  ros::NodeHandle node;//("~");
  JointTrajectoryExecuter jte(node);

  ros::spin();

  return 0;
}

----------------------------


# LLM Response

 ```cpp
/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include <algorithm>
#include <functional>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <pr2_controllers_msgs/action/joint_trajectory.hpp>
#include <pr2_controllers_msgs/msg/joint_trajectory_controller_state.hpp>

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
private:
  using JointTrajectoryAction = pr2_controllers_msgs::action::JointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<JointTrajectoryAction>;
public:
  JointTrajectoryExecuter(std::shared_ptr<rclcpp::Node> n) :
    node_(n),
    has_active_goal_(false)
  {
    node_->declare_parameter("joints", std::vector<std::string>());
    std::vector<std::string> joint_names;
    node_->get_parameter("joints", joint_names);
    
    if (joint_names.empty())
    {
      RCLCPP_FATAL(node_->get_logger(), "No joints given. (namespace: %s)", node_->get_namespace());
      rclcpp::shutdown();
      exit(1);
    }
    
    joint_names_ = joint_names;

    node_->declare_parameter("constraints/goal_time", 0.0);
    node_->get_parameter("constraints/goal_time", goal_time_constraint_);

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns = std::string("constraints/") + joint_names_[i];
      node_->declare_parameter(ns + "/goal", -1.0);
      node_->declare_parameter(ns + "/trajectory", -1.0);
      
      double g, t;
      node_->get_parameter(ns + "/goal", g);
      node_->get_parameter(ns + "/trajectory", t);
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }
    
    node_->declare_parameter("constraints/stopped_velocity_tolerance", 0.01);
    node_->get_parameter("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_);

    action_server_ = rclcpp_action::create_server<JointTrajectoryAction>(
      node_,
      "joint_trajectory_action",
      std::bind(&JointTrajectoryExecuter::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointTrajectoryExecuter::cancelCB, this, std::placeholders::_1),
      std::bind(&JointTrajectoryExecuter::goalCB, this, std::placeholders::_1));

    pub_controller_command_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("command", 1);
    sub_controller_state_ =
      node_->create_subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>(
        "state", 1, std::bind(&JointTrajectoryExecuter::controllerStateCB, this, std::placeholders::_1));

    watchdog_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(1.0), std::bind(&JointTrajectoryExecuter::watchdog, this));

    rclcpp::Time started_waiting_for_controller = node_->now();
    while (rclcpp::ok() && !last_controller_state_)
    {
      rclcpp::spin_some(node_);
      if (started_waiting_for_controller != rclcpp::Time(0, 0, RCL_ROS_TIME) &&
          node_->now() > started_waiting_for_controller + rclcpp::Duration::from_seconds(30.0))
      {
        RCLCPP_WARN(node_->get_logger(), "Waited for the controller for 30 seconds, but it never showed up.");
        started_waiting_for_controller = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  ~JointTrajectoryExecuter()
  {
    pub_controller_command_.reset();
    sub_controller_state_.reset();
    watchdog_timer_->cancel();
  }

private:

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const JointTrajectoryAction::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size() != b.size())
      return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
      if (std::count(b.begin(), b.end(), a[i]) != 1)
        return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
      if (std::count(a.begin(), a.end(), b[i]) != 1)
        return false;
    }

    return true;
  }

  void watchdog()
  {
    rclcpp::Time now = node_->now();

    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        RCLCPP_WARN(node_->get_logger(), "Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > rclcpp::Duration::from_seconds(5.0))
      {
        should_abort = true;
        RCLCPP_WARN(node_->get_logger(), "Aborting goal because we haven't heard from the controller in %.3lf seconds",
                 (now - last_controller_state_->header.stamp).seconds());
      }

      if (should_abort)
      {
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);

        auto result = std::make_shared<JointTrajectoryAction::Result>();
        active_goal_->abort(result);
        has_active_goal_ = false;
      }
    }
  }

  void goalCB(std::shared_ptr<GoalHandle> gh)
  {
    gh->execute();
    
    if (has_active_goal_)
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<JointTrajectoryAction::Result>();
      active_goal_->abort(result);
      has_active_goal_ = false;
    }
    
    active_goal_ = gh;
    has_active_goal_ = true;
    
    current_traj_ = gh->get_goal()->trajectory;
    pub_controller_command_->publish(current_traj_);
  }

  void cancelCB(std::shared_ptr<GoalHandle> gh)
  {
    if (active_goal_ == gh)
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<JointTrajectoryAction::Result>();
      active_goal_->canceled(result);
      has_active_goal_ = false;
    }
  }


  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Server<JointTrajectoryAction>::SharedPtr action_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_controller_state_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  bool has_active_goal_;
  std::shared_ptr<GoalHandle> active_goal_;
  trajectory_msgs::msg::JointTrajectory current_traj_;


  std::vector<std::string> joint_names_;
  std::map<std::string,double> goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  pr2_controllers_msgs::msg::JointTrajectoryControllerState::SharedPtr last_controller_state_;
  void controllerStateCB(const pr2_controllers_msgs::msg::JointTrajectoryControllerState::SharedPtr &msg)
  {
    last_controller_state_ = msg;
    
    if (!has_active_goal_)
      return;
      
    rclcpp::Time now = node_->now();
    
    if (current_traj_.points.empty())
    {
      auto result = std::make_shared<JointTrajectoryAction::Result>();
      active_goal_->succeed(result);
      has_active_goal_ = false;
      return;
    }
    
    rclcpp::Time traj_start_time(current_traj_.header.stamp.sec, current_traj_.header.stamp.nanosec, RCL_ROS_TIME);
    rclcpp::Time traj_end_time = traj_start_time + rclcpp::Duration(
      current_traj_.points.back().time_from_start.sec, 
      current_traj_.points.back().time_from_start.nanosec);
    
    if (now >= traj_end_time)
    {
      bool within_goal_constraints = true;
      
      for (size_t i = 0; i < joint_names_.size() && i < msg->actual.positions.size(); ++i)
      {
        double goal_tolerance = goal_constraints_[joint_names_[i]];
        if (goal_tolerance < 0) continue;
        
        double error = std::abs(msg->actual.positions[i] - current_traj_.points.back().positions[i]);
        if (error > goal_tolerance)
        {
          within_goal_constraints = false;
          break;
        }
      }
      
      bool stopped = true;
      for (size_t i = 0; i < joint_names_.size() && i < msg->actual.velocities.size(); ++i)
      {
        if (std::abs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
        {
          stopped = false;
          break;
        }
      }
      
      if (within_goal_constraints && stopped)
      {
        auto result = std::make_shared<JointTrajectoryAction::Result>();
        active_goal_->succeed(result);
        has_active_goal_ = false;
      }
      else if (now > traj_end_time + rclcpp::Duration::from_seconds(goal_time_constraint_))
      {
        auto result = std::make_shared<JointTrajectoryAction::Result>();
        active_goal_->abort(result);
        has_active_goal_ = false;
        
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);
      }
    }
    else
    {
      for (size_t i = 0; i < joint_names_.size() && i < msg->actual.positions.size(); ++i)
      {
        double traj_tolerance = trajectory_constraints_[joint_names_[i]];
        if (traj_tolerance < 0) continue;
        
        double error = std::abs(msg->actual.positions[i] - current_traj_.points.back().positions[i]);
        if (error > traj_tolerance)
        {
          auto result = std::make_shared<JointTrajectoryAction::Result>();
          active_goal_->abort(result);
          has_active_goal_ = false;
          
          trajectory_msgs::msg::JointTrajectory empty;
          empty.joint_names = joint_names_;
          pub_controller_command_->publish(empty);
          return;
        }
      }
    }
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("joint_trajectory_action_node");
  JointTrajectoryExecuter jte(node);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
```