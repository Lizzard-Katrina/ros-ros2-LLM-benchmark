# Task: action_server/task_003_pr2_gripper_action_server

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

#include <memory>
#include <algorithm>
#include <vector>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "pr2_controllers_msgs/action/joint_trajectory.hpp"
#include "pr2_controllers_msgs/msg/joint_trajectory_controller_state.hpp"

using namespace std::placeholders;

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter : public rclcpp::Node
{
private:
  using JTAS = rclcpp_action::Server<pr2_controllers_msgs::action::JointTrajectory>;
  using GoalHandle = rclcpp_action::ServerGoalHandle<pr2_controllers_msgs::action::JointTrajectory>;

public:
  JointTrajectoryExecuter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("joint_trajectory_action_node", options),
    has_active_goal_(false)
  {
    // Gets all of the joints
    if (!this->get_parameter("joints", joint_names_))
    {
      RCLCPP_FATAL(this->get_logger(), "No joints given. (namespace: %s)", this->get_namespace());
      rclcpp::shutdown();
      exit(1);
    }
    if (joint_names_.empty())
    {
      RCLCPP_FATAL(this->get_logger(), "Malformed joint specification. (namespace: %s)", this->get_namespace());
      rclcpp::shutdown();
      exit(1);
    }

    this->declare_parameter<double>("constraints.goal_time", 0.0);
    this->get_parameter("constraints.goal_time", goal_time_constraint_);

    // Gets the constraints for each joint.
    for (const auto & joint_name : joint_names_)
    {
      std::string goal_param = "constraints." + joint_name + ".goal";
      std::string traj_param = "constraints." + joint_name + ".trajectory";
      double g = -1.0, t = -1.0;
      this->declare_parameter<double>(goal_param, -1.0);
      this->declare_parameter<double>(traj_param, -1.0);
      this->get_parameter(goal_param, g);
      this->get_parameter(traj_param, t);
      goal_constraints_[joint_name] = g;
      trajectory_constraints_[joint_name] = t;
    }
    this->declare_parameter<double>("constraints.stopped_velocity_tolerance", 0.01);
    this->get_parameter("constraints.stopped_velocity_tolerance", stopped_velocity_tolerance_);

    pub_controller_command_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("command", 1);
    sub_controller_state_ = this->create_subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>(
      "state", 1, std::bind(&JointTrajectoryExecuter::controllerStateCB, this, _1));

    watchdog_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&JointTrajectoryExecuter::watchdog, this));

    // Wait for controller state to appear (up to 30 seconds)
    auto start = this->now();
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !last_controller_state_)
    {
      rclcpp::spin_some(this->get_node_base_interface());
      if (start != rclcpp::Time(0) && this->now() > start + rclcpp::Duration::from_seconds(30.0))
      {
        RCLCPP_WARN(this->get_logger(), "Waited for the controller for 30 seconds, but it never showed up.");
        start = rclcpp::Time(0);
      }
      rate.sleep();
    }

    action_server_ = rclcpp_action::create_server<pr2_controllers_msgs::action::JointTrajectory>(
      this,
      "joint_trajectory_action",
      std::bind(&JointTrajectoryExecuter::handle_goal, this, _1, _2),
      std::bind(&JointTrajectoryExecuter::handle_cancel, this, _1),
      std::bind(&JointTrajectoryExecuter::handle_accepted, this, _1));
  }

  ~JointTrajectoryExecuter()
  {
    pub_controller_command_.reset();
    sub_controller_state_.reset();
    watchdog_timer_.reset();
  }

private:

  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size() != b.size())
      return false;

    for (const auto & elem : a)
    {
      if (std::count(b.begin(), b.end(), elem) != 1)
        return false;
    }
    for (const auto & elem : b)
    {
      if (std::count(a.begin(), a.end(), elem) != 1)
        return false;
    }

    return true;
  }

  void watchdog()
  {
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
      else if ((now - last_controller_state_->header.stamp) > rclcpp::Duration::from_seconds(5.0))
      {
        should_abort = true;
        RCLCPP_WARN(this->get_logger(), "Aborting goal because we haven't heard from the controller in %.3lf seconds",
                 (now - last_controller_state_->header.stamp).seconds());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);

        // Marks the current goal as aborted.
        active_goal_->abort(pr2_controllers_msgs::action::JointTrajectory::Result());
        has_active_goal_ = false;
      }
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pr2_controllers_msgs::action::JointTrajectory::Goal> goal)
  {
    (void)uuid;
    // Accept all goals
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (active_goal_ && active_goal_ == goal_handle)
    {
      // Stops the controller.
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      // Marks the current goal as canceled.
      active_goal_->canceled(pr2_controllers_msgs::action::JointTrajectory::Result());
      has_active_goal_ = false;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // This function runs in a separate thread, so we must synchronize access.
    std::lock_guard<std::mutex> lock(goal_mutex_);

    // Cancel any currently active goal
    if (has_active_goal_)
    {
      active_goal_->abort(pr2_controllers_msgs::action::JointTrajectory::Result());
      has_active_goal_ = false;
    }

    // Accept the new goal
    active_goal_ = goal_handle;
    has_active_goal_ = true;

    // Publish the trajectory contained in the goal to the controller.
    trajectory_msgs::msg::JointTrajectory traj = goal_handle->get_goal()->trajectory;

    // Check joint names match
    if (!setsEqual(traj.joint_names, joint_names_))
    {
      RCLCPP_WARN(this->get_logger(), "Received trajectory with joint names that do not match controller joints.");
      active_goal_->abort(pr2_controllers_msgs::action::JointTrajectory::Result());
      has_active_goal_ = false;
      return;
    }

    current_traj_ = traj;
    pub_controller_command_->publish(traj);

    // Mark the new goal as accepted (no explicit accept call in ROS2, just start processing)
  }

  void controllerStateCB(const pr2_controllers_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    last_controller_state_ = msg;

    if (!has_active_goal_)
      return;

    // Track the execution of the currently active trajectory.
    // Check if the controller maintains the trajectory within constraints.
    // If constraints are violated, abort the active goal.
    // If the trajectory reaches the goal within allowed tolerances, mark the goal as succeeded.

    // For each joint, check position and velocity errors against constraints
    bool all_within_goal = true;
    bool any_violation = false;

    const auto & desired_points = current_traj_.points;
    if (desired_points.empty())
      return;

    // Use last point as goal
    const auto & goal_point = desired_points.back();

    // Map joint name to index in current_traj_
    std::map<std::string, size_t> joint_to_index;
    for (size_t i = 0; i < current_traj_.joint_names.size(); ++i)
      joint_to_index[current_traj_.joint_names[i]] = i;

    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
      const std::string & joint = msg->joint_names[i];

      auto it = joint_to_index.find(joint);
      if (it == joint_to_index.end())
      {
        // Joint not in current trajectory, skip
        continue;
      }
      size_t idx = it->second;

      double pos_error = std::abs(goal_point.positions[idx] - msg->desired.positions[i]);
      double vel_error = std::abs(msg->desired.velocities[i] - msg->actual.velocities[i]);

      double goal_constraint = goal_constraints_.count(joint) ? goal_constraints_[joint] : DEFAULT_GOAL_THRESHOLD;
      double traj_constraint = trajectory_constraints_.count(joint) ? trajectory_constraints_[joint] : DEFAULT_GOAL_THRESHOLD;

      // Check trajectory constraint violation (position error)
      if (pos_error > traj_constraint && traj_constraint >= 0.0)
      {
        any_violation = true;
        RCLCPP_WARN(this->get_logger(), "Trajectory constraint violated on joint %s: error %.4f > %.4f",
                    joint.c_str(), pos_error, traj_constraint);
      }

      // Check stopped velocity tolerance if near goal
      if (pos_error < goal_constraint)
      {
        if (std::abs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
        {
          all_within_goal = false;
        }
      }
      else
      {
        all_within_goal = false;
      }
    }

    if (any_violation)
    {
      // Abort the active goal
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      active_goal_->abort(pr2_controllers_msgs::action::JointTrajectory::Result());
      has_active_goal_ = false;
      return;
    }

    if (all_within_goal)
    {
      // Succeeded
      active_goal_->succeed(pr2_controllers_msgs::action::JointTrajectory::Result());
      has_active_goal_ = false;
      return;
    }

    // Otherwise, still executing, can publish feedback if desired (not implemented here)
  }

  JTAS::SharedPtr action_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_controller_state_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex goal_mutex_;

  bool has_active_goal_;
  std::shared_ptr<GoalHandle> active_goal_;
  trajectory_msgs::msg::JointTrajectory current_traj_;

  std::vector<std::string> joint_names_;
  std::map<std::string,double> goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  pr2_controllers_msgs::msg::JointTrajectoryControllerState::SharedPtr last_controller_state_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryExecuter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```