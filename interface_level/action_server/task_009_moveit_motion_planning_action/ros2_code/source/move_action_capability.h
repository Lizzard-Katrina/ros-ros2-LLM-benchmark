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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <memory>
#include <atomic>

namespace move_group
{

enum MoveGroupState
{
  IDLE,
  PLANNING,
  MONITOR,
  LOOK
};

inline std::string stateToStr(MoveGroupState state)
{
  switch (state)
  {
    case IDLE:
      return "IDLE";
    case PLANNING:
      return "PLANNING";
    case MONITOR:
      return "MONITOR";
    case LOOK:
      return "LOOK";
    default:
      return "UNKNOWN";
  }
}

class MoveGroupMoveAction
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using GoalHandleMoveGroup = rclcpp_action::ServerGoalHandle<MoveGroup>;

  MoveGroupMoveAction();

  void initialize();

  std::string getName() const { return name_; }

  rclcpp::Node::SharedPtr getNode() const { return node_; }

private:
  void executeMoveCallback(const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  void executeMoveCallbackPlanAndExecute(
    const std::shared_ptr<const MoveGroup::Goal> goal,
    std::shared_ptr<MoveGroup::Result> & action_res,
    const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  void executeMoveCallbackPlanOnly(
    const std::shared_ptr<const MoveGroup::Goal> goal,
    std::shared_ptr<MoveGroup::Result> & action_res,
    const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  void startMoveExecutionCallback();
  void startMoveLookCallback();
  void preemptMoveCallback();
  void setMoveState(MoveGroupState state);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<MoveGroup>::SharedPtr move_action_server_;
  moveit_msgs::action::MoveGroup::Feedback move_feedback_;

  MoveGroupState move_state_;
  std::atomic<bool> preempt_requested_;
  std::shared_ptr<GoalHandleMoveGroup> current_goal_handle_;
  std::string name_;
};

}  // namespace move_group