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

FILE_PATH: joint_trajectory_action.cpp
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

[FILENAME: joint_trajectory_action.cpp]
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
 * SUBSTITUTE GOODS OR SERVICES, LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
private:
  typedef control_msgs::action::JointTrajectory JointTrajectoryAction;
  typedef rclcpp_action::ServerGoalHandle<JointTrajectoryAction> GoalHandle;
  typedef std::shared_ptr<GoalHandle> GoalHandlePtr;

public:
  JointTrajectoryExecuter(rclcpp::Node::SharedPtr n) :
    node_(n),
    has_active_goal_(false)
  {
    auto pn = node_;

    // Gets all of the joints
    if (!pn->has_parameter("joints"))
    {
      pn->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
    }

    joint_names_ = pn->get_parameter("joints").as_string_array();
    if (joint_names_.empty())
    {
      RCLCPP_FATAL(node_->get_logger(), "No joints given. (namespace: %s)", pn->get_namespace());
      rclcpp::shutdown();
      exit(1);
    }

    if (!pn->has_parameter("constraints.goal_time"))
    {
      pn->declare_parameter<double>("constraints.goal_time", 0.0);
    }
    goal_time_constraint_ = pn->get_parameter("constraints.goal_time").as_double();

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns = std::string("constraints.") + joint_names_[i];

      if (!pn->has_parameter(ns + ".goal"))
      {
        pn->declare_parameter<double>(ns + ".goal", -1.0);
      }
      if (!pn->has_parameter(ns + ".trajectory"))
      {
        pn->declare_parameter<double>(ns + ".trajectory", -1.0);
      }

      double g = pn->get_parameter(ns + ".goal").as_double();
      double t = pn->get_parameter(ns + ".trajectory").as_double();
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }

    if (!pn->has_parameter("constraints.stopped_velocity_tolerance"))
    {
      pn->declare_parameter<double>("constraints.stopped_velocity_tolerance", 0.01);
    }
    stopped_velocity_tolerance_ =
      pn->get_parameter("constraints.stopped_velocity_tolerance").as_double();

    pub_controller_command_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("command", 1);
    sub_controller_state_ =
      node_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "state", 1,
        std::bind(&JointTrajectoryExecuter::controllerStateCB, this, std::placeholders::_1));

    watchdog_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&JointTrajectoryExecuter::watchdog, this));

    rclcpp::Time started_waiting_for_controller = node_->now();
    rclcpp::Rate wait_rate(10.0);
    while (rclcpp::ok() && !last_controller_state_)
    {
      rclcpp::spin_some(node_);
      if (started_waiting_for_controller.nanoseconds() != 0 &&
          node_->now() > started_waiting_for_controller + rclcpp::Duration::from_seconds(30.0))
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Waited for the controller for 30 seconds, but it never showed up.");
        started_waiting_for_controller = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
      }
      wait_rate.sleep();
    }

    action_server_ = rclcpp_action::create_server<JointTrajectoryAction>(
      node_,
      "joint_trajectory_action",
      std::bind(&JointTrajectoryExecuter::goalCB, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&JointTrajectoryExecuter::cancelCB, this, std::placeholders::_1),
      std::bind(&JointTrajectoryExecuter::acceptedCB, this, std::placeholders::_1));
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
        RCLCPP_WARN(node_->get_logger(),
                    "Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > rclcpp::Duration::from_seconds(5.0))
      {
        should_abort = true;
        RCLCPP_WARN(node_->get_logger(),
                    "Aborting goal because we haven't heard from the controller in %.3lf seconds",
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
          result->error_code = control_msgs::action::JointTrajectory_Result::INVALID_GOAL;
          result->error_string = "Controller state timed out";
          active_goal_->abort(result);
        }
        has_active_goal_ = false;
        active_goal_.reset();
      }
    }
  }

  rclcpp_action::GoalResponse goalCB(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const JointTrajectoryAction::Goal> goal)
  {
    if (!setsEqual(goal->trajectory.joint_names, joint_names_))
    {
      RCLCPP_WARN(node_->get_logger(), "Joints on incoming goal don't match our joints");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->trajectory.points.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Incoming goal trajectory was empty");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void acceptedCB(GoalHandlePtr gh)
  {
    // Accept a new goal from the client.
    // Cancel any currently active goal if present.
    if (has_active_goal_ && active_goal_)
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<JointTrajectoryAction::Result>();
      result->error_code = control_msgs::action::JointTrajectory_Result::INVALID_GOAL;
      result->error_string = "Preempted by a new goal";
      active_goal_->canceled(result);
      has_active_goal_ = false;
      active_goal_.reset();
    }

    active_goal_ = gh;
    has_active_goal_ = true;
    current_traj_ = gh->get_goal()->trajectory;

    // Publish the trajectory contained in the goal to the controller.
    pub_controller_command_->publish(current_traj_);

    // The ROS 2 action server has already marked the goal as accepted before invoking this callback.
  }

  rclcpp_action::CancelResponse cancelCB(GoalHandlePtr gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      // Marks the current goal as canceled.
      auto result = std::make_shared<JointTrajectoryAction::Result>();
      result->error_code = control_msgs::action::JointTrajectory_Result::SUCCESSFUL;
      active_goal_->canceled(result);
      has_active_goal_ = false;
      active_goal_.reset();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<JointTrajectoryAction>::SharedPtr action_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_controller_state_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  bool has_active_goal_;
  GoalHandlePtr active_goal_;
  trajectory_msgs::msg::JointTrajectory current_traj_;

  std::vector<std::string> joint_names_;
  std::map<std::string,double> goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  control_msgs::msg::JointTrajectoryControllerState::SharedPtr last_controller_state_;

  void controllerStateCB(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    last_controller_state_ = msg;

    if (!has_active_goal_ || !active_goal_)
      return;

    if (!setsEqual(msg->joint_names, joint_names_))
    {
      RCLCPP_WARN(node_->get_logger(), "Controller state joint names do not match expected joints");
      return;
    }

    if (current_traj_.points.empty())
      return;

    const trajectory_msgs::msg::JointTrajectoryPoint &desired = msg->desired;
    const trajectory_msgs::msg::JointTrajectoryPoint &error = msg->error;
    const trajectory_msgs::msg::JointTrajectoryPoint &actual = msg->actual;

    if (desired.positions.size() != msg->joint_names.size() ||
        error.positions.size() != msg->joint_names.size())
    {
      return;
    }

    bool trajectory_violation = false;
    bool goal_reached = true;

    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
      const std::string &joint = msg->joint_names[i];

      double trajectory_constraint = trajectory_constraints_[joint];
      if (trajectory_constraint >= 0.0 && std::fabs(error.positions[i]) > trajectory_constraint)
      {
        trajectory_violation = true;
        RCLCPP_WARN(node_->get_logger(),
                    "Joint %s violated trajectory constraint: %.6f > %.6f",
                    joint.c_str(), std::fabs(error.positions[i]), trajectory_constraint);
      }

      double goal_constraint = goal_constraints_[joint];
      if (goal_constraint < 0.0)
        goal_constraint = DEFAULT_GOAL_THRESHOLD;

      if (std::fabs(error.positions[i]) > goal_constraint)
        goal_reached = false;

      if (actual.velocities.size() == msg->joint_names.size() &&
          std::fabs(actual.velocities[i]) > stopped_velocity_tolerance_)
      {
        goal_reached = false;
      }
    }

    if (trajectory_violation)
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<JointTrajectoryAction::Result>();
      result->error_code = control_msgs::action::JointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
      result->error_string = "Trajectory constraint violated";
      active_goal_->abort(result);
      has_active_goal_ = false;
      active_goal_.reset();
      return;
    }

    rclcpp::Time now = node_->now();
    rclcpp::Time traj_start(current_traj_.header.stamp, node_->get_clock()->get_clock_type());
    if (traj_start.nanoseconds() == 0)
      traj_start = now;

    const auto &last_point = current_traj_.points.back();
    rclcpp::Duration traj_duration(last_point.time_from_start);
    bool past_goal_time = now >= traj_start + traj_duration;
    bool past_goal_time_limit =
      now >= traj_start + traj_duration + rclcpp::Duration::from_seconds(goal_time_constraint_);

    if (past_goal_time && goal_reached)
    {
      auto result = std::make_shared<JointTrajectoryAction::Result>();
      result->error_code = control_msgs::action::JointTrajectory_Result::SUCCESSFUL;
      active_goal_->succeed(result);
      has_active_goal_ = false;
      active_goal_.reset();
      return;
    }

    if (past_goal_time_limit && !goal_reached)
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<JointTrajectoryAction::Result>();
      result->error_code = control_msgs::action::JointTrajectory_Result::GOAL_TOLERANCE_VIOLATED;
      result->error_string = "Goal constraint violated";
      active_goal_->abort(result);
      has_active_goal_ = false;
      active_goal_.reset();
    }
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_trajectory_action_node");
  JointTrajectoryExecuter jte(node);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}