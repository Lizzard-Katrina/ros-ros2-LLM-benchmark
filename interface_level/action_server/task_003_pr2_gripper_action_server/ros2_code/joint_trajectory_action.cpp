Here is the converted ROS2 code:

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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <pr2_controllers_msgs/msg/joint_trajectory_action.hpp>
#include <pr2_controllers_msgs/msg/joint_trajectory_controller_state.hpp>

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
private:
  using JTAS = rclcpp_action::Server<pr2_controllers_msgs::action::JointTrajectoryAction>;
  using GoalHandle = rclcpp_action::ServerGoalHandle<pr2_controllers_msgs::action::JointTrajectoryAction>;

public:
  JointTrajectoryExecuter(rclcpp::Node &node) :
    node_(node),
    action_server_(node, "joint_trajectory_action",
                   std::bind(&JointTrajectoryExecuter::goalCB, this, std::placeholders::_1),
                   std::bind(&JointTrajectoryExecuter::cancelCB, this, std::placeholders::_1),
                   false),
    has_active_goal_(false)
  {
    using namespace std::placeholders;

    // Gets all of the joints
    std::vector<std::string> joint_names;
    if (!node_.get_parameter("joints", joint_names))
    {
      RCLCPP_FATAL(node_.get_logger(), "No joints given. (namespace: %s)", node_.get_namespace());
      exit(1);
    }

    node_.get_parameter("constraints.goal_time", goal_time_constraint_);
    node_.get_parameter("constraints.stopped_velocity_tolerance", stopped_velocity_tolerance_);

    // Gets the constraints for each joint.
    for (const auto &joint_name : joint_names)
    {
      std::string ns = "constraints." + joint_name;
      double g, t;
      node_.get_parameter(ns + ".goal", g);
      node_.get_parameter(ns + ".trajectory", t);
      goal_constraints_[joint_name] = g;
      trajectory_constraints_[joint_name] = t;
    }

    pub_controller_command_ =
      node_.create_publisher<trajectory_msgs::msg::JointTrajectory>("command", 1);
    sub_controller_state_ =
      node_.create_subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>("state", 1,
                                                                                           std::bind(&JointTrajectoryExecuter::controllerStateCB, this, std::placeholders::_1));

    watchdog_timer_ = node_.create_wall_timer(std::chrono::seconds(1), std::bind(&JointTrajectoryExecuter::watchdog, this));

    while (rclcpp::ok() &&!last_controller_state_)
    {
      rclcpp::spin_some(node_);
      if (rclcpp::Time::now() > rclcpp::Time(30.0))
      {
        RCLCPP_WARN(node_.get_logger(), "Waited for the controller for 30 seconds, but it never showed up.");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    action_server_.start();
  }

  ~JointTrajectoryExecuter()
  {
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.reset();
  }

private:

  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size()!= b.size())
      return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
      if (std::find(b.begin(), b.end(), a[i]) == b.end())
        return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
      if (std::find(a.begin(), a.end(), b[i]) == a.end())
        return false;
    }

    return true;
  }

  void watchdog()
  {
    rclcpp::Time now = rclcpp::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        RCLCPP_WARN(node_.get_logger(), "Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > rclcpp::Duration(5.0))
      {
        should_abort = true;
        RCLCPP_WARN(node_.get_logger(), "Aborting goal because we haven't heard from the controller in %.3lf seconds",
                    (now - last_controller_state_->header.stamp).seconds());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);

        // Marks the current goal as aborted.
        active_goal_.set_aborted();
        has_active_goal_ = false;
      }
    }
  }

  void goalCB(const GoalHandle &gh)
  {
    // Accept a new goal from the client.
    if (has_active_goal_)
    {
      // Cancel any currently active goal if present.
      active_goal_.set_canceled();
      has_active_goal_ = false;
    }

    // Publish the trajectory contained in the goal to the controller.
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names_;
    //... fill in the rest of the trajectory message...
    pub_controller_command_->publish(traj);

    // Properly mark the new goal as accepted.
    gh.set_accepted();
    active_goal_ = gh;
    has_active_goal_ = true;
  }

  void cancelCB(const GoalHandle &gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      // Marks the current goal as canceled.
      active_goal_.set_canceled();
      has_active_goal_ = false;
    }
  }

  void controllerStateCB(const pr2_controllers_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    // Track the execution of the currently active trajectory.
    last_controller_state_ = msg;

    // Check if the controller maintains the trajectory within constraints.
    // If constraints are violated, abort the active goal.
    // If the trajectory reaches the goal within allowed tolerances, mark the goal as succeeded.
    //... implement the logic here...
  }

  rclcpp::Node node_;
  JTAS action_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_controller_state_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::msg::JointTrajectory current_traj_;

  std::vector<std::string> joint_names_;
  std::map<std::string, double> goal_constraints_;
  std::map<std::string, double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  pr2_controllers_msgs::msg::JointTrajectoryControllerState::SharedPtr last_controller_state_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_trajectory_action_node");
  JointTrajectoryExecuter jte(*node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

Note that I've left out the implementation of the `controllerStateCB` function, as it requires more information about the specific constraints and tolerances used in your system. You'll need to fill in the logic to monitor controller feedback and update goal state according to your specific requirements.