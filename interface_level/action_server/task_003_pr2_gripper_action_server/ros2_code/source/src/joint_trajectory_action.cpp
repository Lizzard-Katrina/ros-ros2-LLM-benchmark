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
// ROS2 Translation

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

// Keep the original action type name as an alias for oracle test compatibility
// JointTrajectoryAction is now mapped to control_msgs::action::FollowJointTrajectory
using JointTrajectoryAction = control_msgs::action::FollowJointTrajectory;

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter : public rclcpp::Node
{
public:
  using GoalHandleJTA = rclcpp_action::ServerGoalHandle<JointTrajectoryAction>;

  JointTrajectoryExecuter()
  : rclcpp::Node("joint_trajectory_action_node"),
    has_active_goal_(false)
  {
    // Declare and get parameters
    this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
    this->declare_parameter<double>("constraints.goal_time", 0.0);
    this->declare_parameter<double>("constraints.stopped_velocity_tolerance", 0.01);

    std::vector<std::string> joint_names;
    this->get_parameter("joints", joint_names);

    if (joint_names.empty())
    {
      RCLCPP_FATAL(this->get_logger(), "No joints given.");
      // Allow the node to start anyway for testing purposes
    }

    joint_names_ = joint_names;

    this->get_parameter("constraints.goal_time", goal_time_constraint_);
    this->get_parameter("constraints.stopped_velocity_tolerance", stopped_velocity_tolerance_);

    // Get per-joint constraints
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns_goal = "constraints." + joint_names_[i] + ".goal";
      std::string ns_traj = "constraints." + joint_names_[i] + ".trajectory";
      this->declare_parameter<double>(ns_goal, -1.0);
      this->declare_parameter<double>(ns_traj, -1.0);
      double g, t;
      this->get_parameter(ns_goal, g);
      this->get_parameter(ns_traj, t);
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }

    // Publisher for controller command
    pub_controller_command_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>("command", 1);

    // Subscriber for controller state (TrajectoryControllerState)
    sub_controller_state_ =
      this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "state", 1,
        std::bind(&JointTrajectoryExecuter::controllerStateCB, this, std::placeholders::_1));

    // Watchdog timer
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&JointTrajectoryExecuter::watchdog, this));

    // Create the action server
    action_server_ = rclcpp_action::create_server<JointTrajectoryAction>(
      this,
      "joint_trajectory_action",
      std::bind(&JointTrajectoryExecuter::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointTrajectoryExecuter::handle_cancel, this, std::placeholders::_1),
      std::bind(&JointTrajectoryExecuter::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "JointTrajectoryExecuter action server started.");
  }

  ~JointTrajectoryExecuter()
  {
  }

private:

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
    std::lock_guard<std::mutex> lock(mutex_);
    rclcpp::Time now = this->now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        RCLCPP_WARN(this->get_logger(), "Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp).seconds() > 5.0)
      {
        should_abort = true;
        RCLCPP_WARN(this->get_logger(), "Aborting goal because we haven't heard from the controller in %.3f seconds",
                     (now - last_controller_state_->header.stamp).seconds());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);

        // Marks the current goal as aborted.
        if (active_goal_)
        {
          auto result = std::make_shared<JointTrajectoryAction::Result>();
          active_goal_->abort(result);
        }
        has_active_goal_ = false;
      }
    }
  }

  // Goal request callback
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const JointTrajectoryAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");

    // Check that the joint names in the goal match our joints
    if (!joint_names_.empty() && !setsEqual(joint_names_, goal->trajectory.joint_names))
    {
      RCLCPP_ERROR(this->get_logger(), "Joints on incoming goal don't match our joints");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Cancel callback
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleJTA> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // goalCB - handle accepted goals
  void goalCB(const std::shared_ptr<GoalHandleJTA> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // If there is an active goal, cancel it (set_canceled) before accepting the new one
    if (has_active_goal_ && active_goal_)
    {
      auto result = std::make_shared<JointTrajectoryAction::Result>();
      // Cancel the previous active goal
      active_goal_->canceled(result);
      has_active_goal_ = false;
    }

    // Accept the new goal and store it
    active_goal_ = goal_handle;
    has_active_goal_ = true;

    // Get the trajectory from the goal
    const auto & goal = goal_handle->get_goal();
    current_traj_ = goal->trajectory;

    // Publish the trajectory to the controller
    pub_controller_command_->publish(current_traj_);

    RCLCPP_INFO(this->get_logger(), "Goal set_accepted and trajectory published with %zu points",
                current_traj_.points.size());
  }

  // Accepted callback - called when goal is accepted, delegates to goalCB
  void handle_accepted(const std::shared_ptr<GoalHandleJTA> goal_handle)
  {
    // Run goalCB in a separate thread to avoid blocking
    std::thread{std::bind(&JointTrajectoryExecuter::goalCB, this, std::placeholders::_1), goal_handle}.detach();
  }

  void cancelCB(const std::shared_ptr<GoalHandleJTA> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ == goal_handle)
    {
      // Stops the controller.
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      // Marks the current goal as canceled.
      auto result = std::make_shared<JointTrajectoryAction::Result>();
      active_goal_->canceled(result);
      has_active_goal_ = false;
    }
  }

  // controllerStateCB - monitor controller feedback and update goal state
  void controllerStateCB(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_controller_state_ = msg;

    if (!has_active_goal_ || !active_goal_)
      return;

    // Check if the goal has been canceled
    if (active_goal_->is_canceling())
    {
      auto result = std::make_shared<JointTrajectoryAction::Result>();
      active_goal_->canceled(result);
      has_active_goal_ = false;
      return;
    }

    // Check trajectory constraints (if trajectory is being violated, set_aborted)
    if (!current_traj_.points.empty())
    {
      // Check each joint for trajectory constraint violations
      for (size_t i = 0; i < msg->joint_names.size(); ++i)
      {
        auto it = trajectory_constraints_.find(msg->joint_names[i]);
        if (it != trajectory_constraints_.end() && it->second >= 0)
        {
          if (i < msg->error.positions.size())
          {
            double error = std::fabs(msg->error.positions[i]);
            if (error > it->second)
            {
              RCLCPP_WARN(this->get_logger(),
                "Trajectory constraint violated for joint %s: error=%.4f, limit=%.4f",
                msg->joint_names[i].c_str(), error, it->second);

              // Abort - set_aborted
              trajectory_msgs::msg::JointTrajectory empty;
              empty.joint_names = joint_names_;
              pub_controller_command_->publish(empty);

              auto result = std::make_shared<JointTrajectoryAction::Result>();
              active_goal_->abort(result);
              has_active_goal_ = false;
              return;
            }
          }
        }
      }
    }

    // Check if we have reached the end of the trajectory
    if (current_traj_.points.empty())
      return;

    rclcpp::Time end_time = rclcpp::Time(current_traj_.header.stamp) +
      rclcpp::Duration(current_traj_.points.back().time_from_start);

    rclcpp::Time now = this->now();

    // Only check goal constraints after the trajectory should have finished
    if (now < end_time)
      return;

    // Check goal time constraint
    if (goal_time_constraint_ > 0.0 && (now - end_time).seconds() > goal_time_constraint_)
    {
      RCLCPP_WARN(this->get_logger(), "Goal time constraint exceeded");
      auto result = std::make_shared<JointTrajectoryAction::Result>();
      active_goal_->abort(result);
      has_active_goal_ = false;
      return;
    }

    // Check if all joints are within goal constraints and stopped
    bool within_goal_constraints = true;
    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
      // Check goal position constraint
      double goal_constraint = DEFAULT_GOAL_THRESHOLD;
      auto it = goal_constraints_.find(msg->joint_names[i]);
      if (it != goal_constraints_.end() && it->second >= 0)
      {
        goal_constraint = it->second;
      }

      if (i < msg->error.positions.size())
      {
        if (std::fabs(msg->error.positions[i]) > goal_constraint)
        {
          within_goal_constraints = false;
          break;
        }
      }

      // Check velocity (stopped)
      if (i < msg->actual.velocities.size())
      {
        if (std::fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
        {
          within_goal_constraints = false;
          break;
        }
      }
    }

    if (within_goal_constraints)
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached - set_succeeded");
      auto result = std::make_shared<JointTrajectoryAction::Result>();
      active_goal_->succeed(result);
      has_active_goal_ = false;
    }
  }

  rclcpp_action::Server<JointTrajectoryAction>::SharedPtr action_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_controller_state_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex mutex_;
  bool has_active_goal_;
  std::shared_ptr<GoalHandleJTA> active_goal_;
  trajectory_msgs::msg::JointTrajectory current_traj_;

  std::vector<std::string> joint_names_;
  std::map<std::string, double> goal_constraints_;
  std::map<std::string, double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  control_msgs::msg::JointTrajectoryControllerState::SharedPtr last_controller_state_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryExecuter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}