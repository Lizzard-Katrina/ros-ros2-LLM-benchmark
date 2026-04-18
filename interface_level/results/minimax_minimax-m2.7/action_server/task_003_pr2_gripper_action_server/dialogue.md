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

// Copyright (c) 2009, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author: Stuart Glaser

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <pr2_controllers_msgs/action/joint_trajectory_action.hpp>
#include <pr2_controllers_msgs/msg/joint_trajectory_controller_state.hpp>

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
private:
  typedef rclcpp_action::Server<pr2_controllers_msgs::action::JointTrajectoryAction> JTAS;
  typedef std::shared_ptr<rclcpp_action::ServerGoalHandle<pr2_controllers_msgs::action::JointTrajectoryAction>> GoalHandle;
public:
  JointTrajectoryExecuter(rclcpp::Node::SharedPtr node) :
    node_(node),
    action_server_(rclcpp_action::create_server<JTAS>(
      node_,
      "joint_trajectory_action",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const pr2_controllers_msgs::action::JointTrajectoryAction::Goal> goal) {
        return goalCB(uuid, goal);
      },
      [this](std::shared_ptr<GoalHandle> goal_handle) {
        return cancelCB(goal_handle);
      })),
    has_active_goal_(false)
  {
    rclcpp::Node::SharedPtr pn = rclcpp::Node::make_shared("~");

    // Gets all of the joints
    std::vector<std::string> joint_names_param;
    if (!pn->has_parameter("joints"))
    {
      RCLCPP_FATAL(pn->get_logger(), "No joints given. (namespace: %s)", pn->get_namespace());
      exit(1);
    }
    rclcpp::Parameter joint_names_rclcpp_param;
    pn->get_parameter("joints", joint_names_rclcpp_param);
    if (joint_names_rclcpp_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      RCLCPP_FATAL(pn->get_logger(), "Malformed joint specification.  (namespace: %s)", pn->get_namespace());
      exit(1);
    }
    joint_names_param = joint_names_rclcpp_param.as_string_array();
    for (size_t i = 0; i < joint_names_param.size(); ++i)
    {
      joint_names_.push_back(joint_names_param[i]);
    }

    pn->declare_parameter("constraints.goal_time", rclcpp::ParameterValue(0.0));
    pn->get_parameter("constraints.goal_time", goal_time_constraint_);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns = std::string("constraints.") + joint_names_[i];
      double g = -1.0, t = -1.0;
      pn->declare_parameter(ns + ".goal", rclcpp::ParameterValue(g));
      pn->declare_parameter(ns + ".trajectory", rclcpp::ParameterValue(t));
      pn->get_parameter(ns + ".goal", g);
      pn->get_parameter(ns + ".trajectory", t);
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }
    pn->declare_parameter("constraints.stopped_velocity_tolerance", rclcpp::ParameterValue(0.01));
    pn->get_parameter("constraints.stopped_velocity_tolerance", stopped_velocity_tolerance_);


    pub_controller_command_ =
      node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("command", 1);
    sub_controller_state_ =
      node_->create_subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>(
      "state", 1, std::bind(&JointTrajectoryExecuter::controllerStateCB, this, std::placeholders::_1));

    watchdog_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration(1, 0),
      [this]() { this->watchdog(); });

    rclcpp::Time started_waiting_for_controller = node_->get_clock()->now();
    bool waited_once = false;
    while (rclcpp::ok() && !last_controller_state_)
    {
      rclcpp::spin_some(node_);
      if (!waited_once && node_->get_clock()->now() > started_waiting_for_controller + rclcpp::Duration(30, 0))
      {
        RCLCPP_WARN(node_->get_logger(), "Waited for the controller for 30 seconds, but it never showed up.");
        waited_once = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node_->get_logger(), "Joint trajectory executer started");
  }

  ~JointTrajectoryExecuter()
  {
    watchdog_timer_->cancel();
  }

private:

  rclcpp_action::GoalResponse goalCB(const rclcpp_action::GoalUUID & uuid,
                                     std::shared_ptr<const pr2_controllers_msgs::action::JointTrajectoryAction::Goal> goal)
  {
    (void)uuid;
    if (joint_names_.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "No joints specified, rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Check that joints match
    if (!setsEqual(goal->trajectory.joint_names, joint_names_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint names do not match");
      auto result = std::make_shared<pr2_controllers_msgs::action::JointTrajectoryAction::Result>();
      result->error_code = pr2_controllers_msgs::action::JointTrajectoryAction::Result::INVALID_JOINTS;
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Cancel any currently active goal
    if (has_active_goal_)
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);
      active_goal_->setCanceled(std::make_shared<pr2_controllers_msgs::action::JointTrajectoryAction::Result>());
      has_active_goal_ = false;
    }

    // Store and publish the trajectory
    current_traj_ = goal->trajectory;
    pub_controller_command_->publish(current_traj_);

    // Accept the new goal
    auto goal_handle = rclcpp_action::ServerGoalHandle<pr2_controllers_msgs::action::JointTrajectoryAction>::SharedPtr();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCB(std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
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
    rclcpp::Time now = node_->get_clock()->now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        RCLCPP_WARN(node_->get_logger(), "Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > rclcpp::Duration(5, 0))
      {
        should_abort = true;
        RCLCPP_WARN(node_->get_logger(), "Aborting goal because we haven't heard from the controller in %.3lf seconds",
                 (now - last_controller_state_->header.stamp).seconds());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);

        // Marks the current goal as aborted.
        auto result = std::make_shared<pr2_controllers_msgs::action::JointTrajectoryAction::Result>();
        active_goal_->setAborted(result);
        has_active_goal_ = false;
      }
    }
  }

  bool withinGoalConstraints(const trajectory_msgs::msg::JointTrajectoryPoint & desired,
                             const pr2_controllers_msgs::msg::JointTrajectoryControllerState & state)
  {
    for (size_t i = 0; i < desired.positions.size() && i < state.desired.positions.size(); ++i)
    {
      if (goal_constraints_[joint_names_[i]] > 0 &&
          std::abs(state.desired.positions[i] - desired.positions[i]) > goal_constraints_[joint_names_[i]])
      {
        RCLCPP_ERROR(node_->get_logger(), "Aborting because joint %s exceeded goal constraints",
                     joint_names_[i].c_str());
        return false;
      }
    }
    return true;
  }

  bool withinTrajectoryConstraints(const trajectory_msgs::msg::JointTrajectoryPoint & desired,
                                  const pr2_controllers_msgs::msg::JointTrajectoryControllerState & state)
  {
    for (size_t i = 0; i < desired.positions.size() && i < state.desired.positions.size(); ++i)
    {
      if (trajectory_constraints_[joint_names_[i]] > 0 &&
          std::abs(state.actual.positions[i] - desired.positions[i]) > trajectory_constraints_[joint_names_[i]])
      {
        RCLCPP_ERROR(node_->get_logger(), "Aborting because joint %s violated trajectory constraints",
                     joint_names_[i].c_str());
        return false;
      }
    }
    return true;
  }

  bool trajectoryTimeIsZero(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    for (size_t i = 0; i < traj.points.size(); ++i)
    {
      if (traj.points[i].time_from_start.sec != 0 || traj.points[i].time_from_start.nanosec != 0)
        return false;
    }
    return true;
  }

  bool checkGoalConstraints(const pr2_controllers_msgs::msg::JointTrajectoryControllerState & state)
  {
    if (current_traj_.points.empty())
      return true;

    if (trajectoryTimeIsZero(current_traj_))
    {
      for (size_t i = 0; i < current_traj_.joint_names.size(); ++i)
      {
        double error = std::abs(state.actual.positions[i] - current_traj_.joint_names[i].length());
        if (goal_constraints_[current_traj_.joint_names[i]] > 0 && error > goal_constraints_[current_traj_.joint_names[i]])
        {
          RCLCPP_ERROR(node_->get_logger(), "Aborting because joint %s exceeded goal constraints",
                       current_traj_.joint_names[i].c_str());
          return false;
        }
      }
    }
    else
    {
      return withinGoalConstraints(state.desired, state);
    }
    return true;
  }

  void controllerStateCB(const pr2_controllers_msgs::msg::JointTrajectoryControllerStateConstPtr &msg)
  {
    last_controller_state_ = msg;

    if (!has_active_goal_)
      return;

    if (has_active_goal_ && msg->joint_names.size() != current_traj_.joint_names.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Ignoring controller state message due to mismatched joint names");
      return;
    }

    // Check goal constraints
    if (!checkGoalConstraints(*msg))
    {
      trajectory_msgs::msg::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_->publish(empty);

      auto result = std::make_shared<pr2_controllers_msgs::action::JointTrajectoryAction::Result>();
      result->error_code = pr2_controllers_msgs::action::JointTrajectoryAction::Result::PATH_TOLERANCE_VIOLATED;
      active_goal_->setAborted(result);
      has_active_goal_ = false;
      return;
    }

    // Check if trajectory is complete
    if (current_traj_.points.empty())
      return;

    if (!trajectoryTimeIsZero(current_traj_) && !msg->desired.positions.empty())
    {
      if (!withinGoalConstraints(msg->desired, *msg))
      {
        trajectory_msgs::msg::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_->publish(empty);

        auto result = std::make_shared<pr2_controllers_msgs::action::JointTrajectoryAction::Result>();
        result->error_code = pr2_controllers_msgs::action::JointTrajectoryAction::Result::GOAL_TOLERANCE_VIOLATED;
        active_goal_->setAborted(result);
        has_active_goal_ = false;
        return;
      }

      rclcpp::Duration last_time = rclcpp::Duration(current_traj_.points.back().time_from_start);
      rclcpp::Time now = node_->get_clock()->now();
      rclcpp::Time start_time = rclcpp::Time(last_controller_state_->header.stamp) - rclcpp::Duration(msg->actual.time_from_start);

      if (start_time + last_time + rclcpp::Duration(goal_time_constraint_, 0) < now)
      {
        RCLCPP_INFO(node_->get_logger(), "Trajectory duration exceeded. Succeeding.");
        auto result = std::make_shared<pr2_controllers_msgs::action::JointTrajectoryAction::Result>();
        active_goal_->setSucceeded(result);
        has_active_goal_ = false;
      }
    }

    // Check stopped constraints
    if (has_active_goal_)
    {
      for (size_t i = 0; i < msg->actual.velocities.size(); ++i)
      {
        if (stopped_velocity_tolerance_ > 0 &&
            std::abs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
        {
          return;
        }
      }

      if (!msg->desired.positions.empty())
      {
        if (!withinGoalConstraints(msg->desired, *msg))
        {
          trajectory_msgs::msg::JointTrajectory empty;
          empty.joint_names = joint_names_;
          pub_controller_command_->publish(empty);

          auto result = std::make_shared<pr2_controllers_msgs::action::JointTrajectoryAction::Result>();
          result->error_code = pr2_controllers_msgs::action::JointTrajectoryAction::Result::GOAL_TOLERANCE_VIOLATED;
          active_goal_->setAborted(result);
          has_active_goal_ = false;
        }
      }

      // Check if all points have been processed
      if (!msg->desired.positions.empty())
      {
        bool within_tolerance = true;
        for (size_t i = 0; i < msg->desired.positions.size(); ++i)
        {
          double error = std::abs(state.actual.positions[i] - state.desired.positions[i]);
          if (goal_constraints_[joint_names_[i]] > 0 && error > goal_constraints_[joint_names_[i]])
          {
            within_tolerance = false;
            break;
          }
        }
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<JTAS> action_server_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<pr2_controllers_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_controller_state_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  bool has_active_goal_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<pr2_controllers_msgs::action::JointTrajectoryAction>> active_goal_;
  trajectory_msgs::msg::JointTrajectory current_traj_;


  std::vector<std::string> joint_names_;
  std::map<std::string,double> goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  pr2_controllers_msgs::msg::JointTrajectoryControllerStateConstPtr last_controller_state_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_trajectory_action_node");
  JointTrajectoryExecuter jte(node);

  rclcpp::spin(node);

  return 0;
}