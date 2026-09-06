/*********************************************************************
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>

namespace move_group
{

class MoveGroupMoveAction
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using GoalHandleMoveGroup = rclcpp_action::ServerGoalHandle<MoveGroup>;

  MoveGroupMoveAction()
    : move_state_(IDLE), preempt_requested_{false}, name_("MoveAction")
  {
    node_ = std::make_shared<rclcpp::Node>("move_group_move_action");
  }

  void initialize()
  {
    // start the move action server
    move_action_server_ = rclcpp_action::create_server<moveit_msgs::action::MoveGroup>(
      node_,
      "move_action",
      [this](const rclcpp_action::GoalUUID & uuid,
             std::shared_ptr<const MoveGroup::Goal> goal) {
        return handleGoal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandleMoveGroup> goal_handle) {
        return handleCancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandleMoveGroup> goal_handle) {
        handleAccepted(goal_handle);
      });
  }

  std::string getName() const { return name_; }
  rclcpp::Node::SharedPtr getNode() const { return node_; }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const MoveGroup::Goal> /*goal*/)
  {
    RCLCPP_INFO(node_->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleMoveGroup> /*goal_handle*/)
  {
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    preemptMoveCallback();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
    const std::shared_ptr<GoalHandleMoveGroup> goal_handle)
  {
    current_goal_handle_ = goal_handle;
    std::thread{[this, goal_handle]() {
      executeMoveCallback(goal_handle);
    }}.detach();
  }

  void executeMoveCallback(const std::shared_ptr<GoalHandleMoveGroup> goal_handle)
  {
    setMoveState(PLANNING);

    const auto goal = goal_handle->get_goal();

    if (preempt_requested_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "Preempt requested before planning.");
      auto result = std::make_shared<MoveGroup::Result>();
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      goal_handle->canceled(result);
      setMoveState(IDLE);
      preempt_requested_ = false;
      return;
    }

    auto result = std::make_shared<MoveGroup::Result>();

    if (goal->planning_options.plan_only)
    {
      executeMoveCallbackPlanOnly(goal, result);
    }
    else
    {
      executeMoveCallbackPlanAndExecute(goal, result);
    }

    if (goal_handle->is_canceling())
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      goal_handle->canceled(result);
    }
    else if (result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }

    setMoveState(IDLE);
    preempt_requested_ = false;
  }

  void executeMoveCallbackPlanAndExecute(
    const std::shared_ptr<const MoveGroup::Goal> & /*goal*/,
    std::shared_ptr<MoveGroup::Result> & action_res)
  {
    RCLCPP_INFO(node_->get_logger(),
      "Combined planning and execution request received for MoveGroup action.");

    if (preempt_requested_.load())
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      return;
    }

    setMoveState(MONITOR);
    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  void executeMoveCallbackPlanOnly(
    const std::shared_ptr<const MoveGroup::Goal> & /*goal*/,
    std::shared_ptr<MoveGroup::Result> & action_res)
  {
    RCLCPP_INFO(node_->get_logger(),
      "Planning request received for MoveGroup action.");

    if (preempt_requested_.load())
    {
      action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      return;
    }

    action_res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  void startMoveExecutionCallback()
  {
    setMoveState(MONITOR);
  }

  void startMoveLookCallback()
  {
    setMoveState(LOOK);
  }

  void preemptMoveCallback()
  {
    preempt_requested_ = true;
    RCLCPP_INFO(node_->get_logger(), "Preempt requested");
  }

  void setMoveState(MoveGroupState state)
  {
    move_state_ = state;
    move_feedback_.state = stateToStr(state);

    if (current_goal_handle_)
    {
      auto feedback = std::make_shared<MoveGroup::Feedback>(move_feedback_);
      current_goal_handle_->publish_feedback(feedback);
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<MoveGroup>::SharedPtr move_action_server_;
  std::shared_ptr<GoalHandleMoveGroup> current_goal_handle_;
  moveit_msgs::action::MoveGroup::Feedback move_feedback_;

  MoveGroupState move_state_;
  std::atomic<bool> preempt_requested_;
  std::string name_;
};

}  // namespace move_group