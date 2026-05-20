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

FILE_PATH: move_action_capability.cpp
----------------------------
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

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/message_checks.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupMoveAction::MoveGroupMoveAction()
  : MoveGroupCapability("MoveAction"), move_state_(IDLE), preempt_requested_{ false }
{
}

void MoveGroupMoveAction::initialize()
{
  // start the move action server
  move_action_server_ = std::make_unique<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>>(
      root_node_handle_, MOVE_ACTION, [this](const auto& goal) { executeMoveCallback(goal); }, false);
  move_action_server_->registerPreemptCallback([this] { preemptMoveCallback(); });
  move_action_server_->start();
}

void MoveGroupMoveAction::executeMoveCallback(const moveit_msgs::MoveGroupGoalConstPtr& goal)
{
// TODO:
  // 1. Transition the action server into a planning state and publish feedback.
  // 2. Check whether a preempt or cancel request has been issued
  // 3. Simulate a motion planning result:
  // 4. Set the appropriate result and terminal state.
  // 5. Reset the internal state
//END OF TODO
}

void MoveGroupMoveAction::executeMoveCallbackPlanAndExecute(const moveit_msgs::MoveGroupGoalConstPtr& goal,
                                                            moveit_msgs::MoveGroupResult& action_res)
{
  ROS_INFO_NAMED(getName(), "Combined planning and execution request received for MoveGroup action. "
                            "Forwarding to planning and execution pipeline.");

  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::MotionPlanRequest& motion_plan_request =
      moveit::core::isEmpty(goal->request.start_state) ? goal->request : clearRequestStartState(goal->request);
  const moveit_msgs::PlanningScene& planning_scene_diff =
      moveit::core::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
          goal->planning_options.planning_scene_diff :
          clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = [this] { startMoveExecutionCallback(); };

  opt.plan_callback_ = [this, &motion_plan_request](plan_execution::ExecutableMotionPlan& plan) {
    return planUsingPlanningPipeline(motion_plan_request, plan);
  };
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = [plan_with_sensing = context_->plan_with_sensing_.get(), planner = opt.plan_callback_,
                          attempts = goal->planning_options.look_around_attempts,
                          safe_execution_cost = goal->planning_options.max_safe_execution_cost](
                             plan_execution::ExecutableMotionPlan& plan) {
      return plan_with_sensing->computePlan(plan, planner, attempts, safe_execution_cost);
    };
    context_->plan_with_sensing_->setBeforeLookCallback([this] { return startMoveLookCallback(); });
  }

  plan_execution::ExecutableMotionPlan plan;
  if (preempt_requested_)
  {
    ROS_INFO_NAMED(getName(), "Preempt requested before the goal is planned and executed.");
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    return;
  }

  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
  if (plan.executed_trajectory_)
    plan.executed_trajectory_->getRobotTrajectoryMsg(action_res.executed_trajectory);
  action_res.error_code = plan.error_code_;
}

void MoveGroupMoveAction::executeMoveCallbackPlanOnly(const moveit_msgs::MoveGroupGoalConstPtr& goal,
                                                      moveit_msgs::MoveGroupResult& action_res)
{
  ROS_INFO_NAMED(getName(), "Planning request received for MoveGroup action. Forwarding to planning pipeline.");

  planning_interface::MotionPlanResponse res;

  if (preempt_requested_)
  {
    ROS_INFO_NAMED(getName(), "Preempt requested before the goal is planned.");
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    return;
  }

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(goal->request.pipeline_id);
  if (!planning_pipeline)
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return;
  }

  try
  {
    auto scene = context_->planning_scene_monitor_->copyPlanningScene(goal->planning_options.planning_scene_diff);
    planning_pipeline->generatePlan(scene, goal->request, res);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(getName(), "Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);
  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}

bool MoveGroupMoveAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest& req,
                                                    plan_execution::ExecutableMotionPlan& plan)
{
  setMoveState(PLANNING);

  bool solved = false;
  planning_interface::MotionPlanResponse res;

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(req.pipeline_id);
  if (!planning_pipeline)
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return solved;
  }

  try
  {
    solved = planning_pipeline->generatePlan(plan.copyPlanningScene(), req, res);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(getName(), "Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan.plan_components_.resize(1);
    plan.plan_components_[0].trajectory_ = res.trajectory_;
    plan.plan_components_[0].description_ = "plan";
  }
  plan.error_code_ = res.error_code_;
  return solved;
}

void MoveGroupMoveAction::startMoveExecutionCallback()
{
  setMoveState(MONITOR);
}

void MoveGroupMoveAction::startMoveLookCallback()
{
  setMoveState(LOOK);
}

void MoveGroupMoveAction::preemptMoveCallback()
{
// TODO:
  // Mark the current goal as preempted or canceled
  // and ensure ongoing execution is stopped.
//END OF TODO

}

void MoveGroupMoveAction::setMoveState(MoveGroupState state)
{
// TODO:
  // Update internal state and publish action feedback
  // reflecting the current MoveGroupState.
//END OF TODO

}
}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupMoveAction, move_group::MoveGroupCapability)

----------------------------


# LLM Response

[FILENAME: move_action_capability.cpp]
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

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/message_checks.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupMoveAction::MoveGroupMoveAction()
  : MoveGroupCapability("MoveAction"), move_state_(IDLE), preempt_requested_{ false }
{
}

void MoveGroupMoveAction::initialize()
{
  // start the move action server
  move_action_server_ = std::make_unique<rclcpp_action::Server<moveit_msgs::action::MoveGroup>>(
      node_handle_, MOVE_ACTION,
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const moveit_msgs::action::MoveGroup::Goal> goal) {
        (void)uuid;
        if (preempt_requested_)
        {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>> goal_handle) {
        if (goal_handle->is_active())
        {
          preempt_requested_ = true;
        }
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>> goal_handle) {
        executeMoveCallback(goal_handle);
      });
}

void MoveGroupMoveAction::executeMoveCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::MoveGroup>> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<moveit_msgs::action::MoveGroup::Result>();
  auto feedback = std::make_shared<moveit_msgs::action::MoveGroup::Feedback>();

  setMoveState(PLANNING);
  feedback->state = "PLANNING";
  goal_handle->publish_feedback(feedback);

  if (goal_handle->is_canceling() || preempt_requested_)
  {
    result->error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    goal_handle->canceled(result);
    preempt_requested_ = false;
    setMoveState(IDLE);
    return;
  }

  if (goal->planning_options.plan_only)
  {
    executeMoveCallbackPlanOnly(goal, *result);
  }
  else
  {
    executeMoveCallbackPlanAndExecute(goal, *result);
  }

  if (goal_handle->is_canceling() || preempt_requested_)
  {
    result->error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    goal_handle->canceled(result);
    preempt_requested_ = false;
    setMoveState(IDLE);
    return;
  }

  if (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    goal_handle->succeed(result);
  }
  else
  {
    goal_handle->abort(result);
  }

  preempt_requested_ = false;
  setMoveState(IDLE);
}

void MoveGroupMoveAction::executeMoveCallbackPlanAndExecute(const std::shared_ptr<const moveit_msgs::action::MoveGroup::Goal> goal,
                                                            moveit_msgs::action::MoveGroup::Result& action_res)
{
  RCLCPP_INFO(node_handle_->get_logger(), "Combined planning and execution request received for MoveGroup action. "
                            "Forwarding to planning and execution pipeline.");

  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::MotionPlanRequest& motion_plan_request =
      moveit::core::isEmpty(goal->request.start_state) ? goal->request : clearRequestStartState(goal->request);
  const moveit_msgs::PlanningScene& planning_scene_diff =
      moveit::core::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
          goal->planning_options.planning_scene_diff :
          clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = [this] { startMoveExecutionCallback(); };

  opt.plan_callback_ = [this, &motion_plan_request](plan_execution::ExecutableMotionPlan& plan) {
    return planUsingPlanningPipeline(motion_plan_request, plan);
  };
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = [plan_with_sensing = context_->plan_with_sensing_.get(), planner = opt.plan_callback_,
                          attempts = goal->planning_options.look_around_attempts,
                          safe_execution_cost = goal->planning_options.max_safe_execution_cost](
                             plan_execution::ExecutableMotionPlan& plan) {
      return plan_with_sensing->computePlan(plan, planner, attempts, safe_execution_cost);
    };
    context_->plan_with_sensing_->setBeforeLookCallback([this] { return startMoveLookCallback(); });
  }

  plan_execution::ExecutableMotionPlan plan;
  if (preempt_requested_)
  {
    RCLCPP_INFO(node_handle_->get_logger(), "Preempt requested before the goal is planned and executed.");
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    return;
  }

  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
  if (plan.executed_trajectory_)
    plan.executed_trajectory_->getRobotTrajectoryMsg(action_res.executed_trajectory);
  action_res.error_code = plan.error_code_;
}

void MoveGroupMoveAction::executeMoveCallbackPlanOnly(const std::shared_ptr<const moveit_msgs::action::MoveGroup::Goal> goal,
                                                      moveit_msgs::action::MoveGroup::Result& action_res)
{
  RCLCPP_INFO(node_handle_->get_logger(), "Planning request received for MoveGroup action. Forwarding to planning pipeline.");

  planning_interface::MotionPlanResponse res;

  if (preempt_requested_)
  {
    RCLCPP_INFO(node_handle_->get_logger(), "Preempt requested before the goal is planned.");
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    return;
  }

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(goal->request.pipeline_id);
  if (!planning_pipeline)
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return;
  }

  try
  {
    auto scene = context_->planning_scene_monitor_->copyPlanningScene(goal->planning_options.planning_scene_diff);
    planning_pipeline->generatePlan(scene, goal->request, res);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(node_handle_->get_logger(), "Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);
  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}

bool MoveGroupMoveAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest& req,
                                                    plan_execution::ExecutableMotionPlan& plan)
{
  setMoveState(PLANNING);

  bool solved = false;
  planning_interface::MotionPlanResponse res;

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(req.pipeline_id);
  if (!planning_pipeline)
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return solved;
  }

  try
  {
    solved = planning_pipeline->generatePlan(plan.copyPlanningScene(), req, res);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(node_handle_->get_logger(), "Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan.plan_components_.resize(1);
    plan.plan_components_[0].trajectory_ = res.trajectory_;
    plan.plan_components_[0].description_ = "plan";
  }
  plan.error_code_ = res.error_code_;
  return solved;
}

void MoveGroupMoveAction::startMoveExecutionCallback()
{
  setMoveState(MONITOR);
}

void MoveGroupMoveAction::startMoveLookCallback()
{
  setMoveState(LOOK);
}

void MoveGroupMoveAction::preemptMoveCallback()
{
  preempt_requested_ = true;
  if (context_->plan_execution_)
  {
    context_->plan_execution_->stop();
  }
}

void MoveGroupMoveAction::setMoveState(MoveGroupState state)
{
  move_state_ = state;
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupMoveAction, move_group::MoveGroupCapability)