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
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <pr2_controllers_msgs/action/joint_trajectory.hpp>
#include <pr2_controllers_msgs/msg/joint_trajectory_controller_state.hpp>

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
private:
  typedef pr2_controllers_msgs::action::JointTrajectory JTAction;
  typedef rclcpp_action::Server<JTAction> JTAS;
  typedef rclcpp_action::ServerGoalHandle<JTAction> GoalHandle;

public:
  JointTrajectoryExecuter(rclcpp::Node::SharedPtr &n)
  : node_(n),
    has_active_goal_(false),
    trajectory_start_time_(0, 0, node_->get_clock()->get_clock_type())
  {
    // Gets all of the joints
    node_->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    if (!node_->get_parameter("joints", joint_names_) || joint_names_.empty())
    {
      RCLCPP_FATAL(node_->get_logger(), "No joints given. (node: %s)", node_->get_fully_qualified_name());
      throw std::runtime_error("No joints parameter");
    }

    node_->declare_parameter<double>("constraints.goal_time", 0.0);
    node_->get_parameter("constraints.goal_time", goal_time_constraint_);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      const std::string ns = std::string("constraints.") + joint_names_[i];
      const std::string goal_param = ns + ".goal";
      const std::string traj_param = ns + ".trajectory";
      node_->declare_parameter<double>(goal_param, -1.0);
      node_->declare_parameter<double>(traj_param, -1.0);

      double g = -1.0;
      double t = -1.0;
      node_->get_parameter(goal_param, g);
      node_->get_parameter(traj_param, t);
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }

    node_->declare_parameter<double>("constraints.stopped_velocity_tolerance", 0.01);
    node_->get_parameter("constraints.stopped_velocity_tolerance", stopped_velocity_tolerance_);

    pub_controller_command_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("command", rclcpp::QoS(1));

    sub_controller_state_ =
      node_->create_subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>(
        "state", rclcpp::QoS(1),
        std::bind(&JointTrajectoryExecuter::controllerStateCB, this, std::placeholders::_1));

    watchdog_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&JointTrajectoryExecuter::watchdog, this));

    action_server_ = rclcpp_action::create_server<JTAction>(
      node_,
      "joint_trajectory_action",
      std::bind(&JointTrajectoryExecuter::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointTrajectoryExecuter::handle_cancel, this, std::placeholders::_1),
      std::bind(&JointTrajectoryExecuter::handle_accepted, this, std::placeholders::_1));

    rclcpp::Time started_waiting_for_controller = node_->now();
    while (rclcpp::ok() && !last_controller_state_)
    {
      rclcpp::spin_some(node_);
      if (started_waiting_for_controller.nanoseconds() != 0 &&
          node_->now() > started_waiting_for_controller + rclcpp::Duration::from_seconds(30.0))
      {
        RCLCPP_WARN(node_->get_logger(), "Waited for the controller for 30 seconds, but it never showed up.");
        started_waiting_for_controller = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  ~JointTrajectoryExecuter()
  {
    pub_controller_command_.reset();
    sub_controller_state_.reset();
    watchdog_timer_.reset();
    action_server_.reset();
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

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const JTAction::Goal> goal)
  {
    if (!setsEqual(goal->trajectory.joint_names, joint_names_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint names in goal do not match controller joints.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    cancelCB(goal_handle);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    goalCB(goal_handle);
  }

  void watchdog()
  {
    rclcpp::Time now = node_->now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        RCLCPP_WARN(node_->get_logger(), "Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - rclcpp::Time(last_controller_state_->header.stamp)) > rclcpp::Duration::from_seconds(5.0))
      {
        should_abort = true;
        RCLCPP_WARN(
          node_->get_logger(),
          "Aborting goal because we haven't heard from the controller in %.3lf seconds",
          (now - rclcpp::Time(last_controller_state_->header.stamp)).seconds());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);

        // Marks the current goal as aborted.
        auto result = std::make_shared<JTAction::Result>();
        active_goal_->abort(result);
        has_active_goal_ = false;
      }
    }
  }

  void goalCB(std::shared_ptr<GoalHandle> gh)
  {
    if (!gh) {
      return;
    }

    // Cancel any currently active goal if present.
    if (has_active_goal_ && active_goal_)
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<JTAction::Result>();
      active_goal_->abort(result);
      has_active_goal_ = false;
    }

    // Accept and track new goal.
    active_goal_ = gh;
    has_active_goal_ = true;
    current_traj_ = gh->get_goal()->trajectory;

    if (current_traj_.header.stamp.sec == 0 && current_traj_.header.stamp.nanosec == 0)
      trajectory_start_time_ = node_->now();
    else
      trajectory_start_time_ = rclcpp::Time(current_traj_.header.stamp);

    // Publish trajectory to controller.
    pub_controller_command_->publish(current_traj_);
  }

  void cancelCB(std::shared_ptr<GoalHandle> gh)
  {
    if (has_active_goal_ && active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      // Marks the current goal as canceled.
      auto result = std::make_shared<JTAction::Result>();
      active_goal_->canceled(result);
      has_active_goal_ = false;
    }
  }

  rclcpp::Node::SharedPtr node_;
  JTAS::SharedPtr action_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_controller_state_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  bool has_active_goal_;
  std::shared_ptr<GoalHandle> active_goal_;
  trajectory_msgs::msg::JointTrajectory current_traj_;
  rclcpp::Time trajectory_start_time_;

  std::vector<std::string> joint_names_;
  std::map<std::string, double> goal_constraints_;
  std::map<std::string, double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  pr2_controllers_msgs::msg::JointTrajectoryControllerState::SharedPtr last_controller_state_;

  void controllerStateCB(const pr2_controllers_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    last_controller_state_ = msg;

    if (!has_active_goal_ || !active_goal_)
      return;

    if (!setsEqual(msg->joint_names, joint_names_))
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<JTAction::Result>();
      active_goal_->abort(result);
      has_active_goal_ = false;
      return;
    }

    // Check trajectory constraints while trajectory is executing.
    for (size_t i = 0; i < msg->joint_names.size() && i < msg->error.positions.size(); ++i)
    {
      const std::string &jn = msg->joint_names[i];
      const double c = trajectory_constraints_[jn];
      if (c > 0.0 && std::fabs(msg->error.positions[i]) > c)
      {
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);

        auto result = std::make_shared<JTAction::Result>();
        active_goal_->abort(result);
        has_active_goal_ = false;
        return;
      }
    }

    if (current_traj_.points.empty())
    {
      auto result = std::make_shared<JTAction::Result>();
      active_goal_->succeed(result);
      has_active_goal_ = false;
      return;
    }

    const rclcpp::Time now(msg->header.stamp);
    const rclcpp::Duration traj_duration(current_traj_.points.back().time_from_start);
    const rclcpp::Time end_time = trajectory_start_time_ + traj_duration;
    const rclcpp::Time abort_time = end_time + rclcpp::Duration::from_seconds(goal_time_constraint_);

    bool goal_reached = true;
    for (size_t i = 0; i < msg->joint_names.size() && i < msg->error.positions.size(); ++i)
    {
      const std::string &jn = msg->joint_names[i];
      double goal_tol = goal_constraints_[jn];
      if (goal_tol < 0.0)
        goal_tol = DEFAULT_GOAL_THRESHOLD;

      if (std::fabs(msg->error.positions[i]) > goal_tol)
      {
        goal_reached = false;
        break;
      }

      if (i < msg->actual.velocities.size() &&
          std::fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
      {
        goal_reached = false;
        break;
      }
    }

    if (now >= end_time && goal_reached)
    {
      auto result = std::make_shared<JTAction::Result>();
      active_goal_->succeed(result);
      has_active_goal_ = false;
      return;
    }

    if (now > abort_time && !goal_reached)
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<JTAction::Result>();
      active_goal_->abort(result);
      has_active_goal_ = false;
      return;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("joint_trajectory_action_node");
  JointTrajectoryExecuter jte(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}