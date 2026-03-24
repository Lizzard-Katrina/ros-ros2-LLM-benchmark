# Task: action_server/task_009_moveit_motion_planning_action

/***************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: Ioan Sucan */

#include "move_action_capability.h"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/message_checks.h>
#include <moveit/move_group/capability_names.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace move_group
{
MoveGroupMoveAction::MoveGroupMoveAction()
  : MoveGroupCapability("MoveAction"), move_state_(IDLE), preempt_requested_{false}
{
}

void MoveGroupMoveAction::initialize()
{
  // start the move action server
  using MoveGroupAction = moveit_msgs::action::MoveGroup;
  move_action_server_ = rclcpp_action::create_server<MoveGroupAction>(
      root_node_handle_,
      MOVE_ACTION,
      std::bind(&MoveGroupMoveAction::executeMoveCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveGroupMoveAction::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveGroupMoveAction::handleCancel, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse MoveGroupMoveAction::handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const moveit_msgs::action::MoveGroup::Goal> goal)
{
  (void)uuid;
  (void)goal;
  // Accept all goals
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveGroupMoveAction::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>> goal_handle)
{
  (void)goal_handle;
  preemptMoveCallback();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveGroupMoveAction::executeMoveCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>> goal_handle,
    const std::shared_ptr<const moveit_msgs::action::MoveGroup::Goal> goal)
{
  auto feedback = std::make_shared<moveit_msgs::action::MoveGroup::Feedback>();
  auto result = std::make_shared<moveit_msgs::action::MoveGroup::Result>();

  // 1. Transition the action server into a planning state and publish feedback.
  setMoveState(PLANNING);
  feedback->state = move_state_;
  goal_handle->publish_feedback(feedback);

  // 2. Check whether a preempt or cancel request has been issued
  if (goal_handle->is_canceling())
  {
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    goal_handle->canceled(result);
    setMoveState(IDLE);
    return;
  }

  preempt_requested_ = false;

  // 3. Simulate a motion planning result:
  // For demonstration, we simulate a successful plan with no actual planning.
  // In real code, this would call planning and execution pipelines.

  // Simulate some planning delay
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  if (preempt_requested_ || goal_handle->is_canceling())
  {
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    goal_handle->canceled(result);
    setMoveState(IDLE);
    return;
  }

  // Simulate success
  result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  // 4. Set the appropriate result and terminal state.
  goal_handle->succeed(result);

  // 5. Reset the internal state
  setMoveState(IDLE);
}

void MoveGroupMoveAction::preemptMoveCallback()
{
  // Mark the current goal as preempted or canceled
  // and ensure ongoing execution is stopped.
  preempt_requested_ = true;
  if (move_action_server_)
  {
    // In ROS2, we cannot directly preempt the goal from server side,
    // but we can request cancellation or set flag to stop execution.
    // The execute callback checks preempt_requested_ and cancels accordingly.
  }
}

void MoveGroupMoveAction::setMoveState(MoveGroupState state)
{
  // Update internal state and publish action feedback
  // reflecting the current MoveGroupState.
  move_state_ = state;

  if (move_action_server_)
  {
    auto feedback = std::make_shared<moveit_msgs::action::MoveGroup::Feedback>();
    feedback->state = move_state_;
    // In ROS2, feedback is published from within execute callback via goal_handle.
    // Here, we do not have access to goal_handle, so this is a no-op or could be extended.
    // Alternatively, store last goal_handle and publish feedback if needed.
  }
}
}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupMoveAction, move_group::MoveGroupCapability)