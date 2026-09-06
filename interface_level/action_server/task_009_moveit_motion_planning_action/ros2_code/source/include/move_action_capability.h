/*********************************************************************
 * ROS2 translation of move_action_capability.h
 *********************************************************************/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <memory>
#include <atomic>
#include <string>

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
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveGroup::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  void handleAccepted(
    const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  void executeMoveCallback(const std::shared_ptr<GoalHandleMoveGroup> goal_handle);

  void executeMoveCallbackPlanAndExecute(
    const std::shared_ptr<const MoveGroup::Goal> & goal,
    std::shared_ptr<MoveGroup::Result> & action_res);

  void executeMoveCallbackPlanOnly(
    const std::shared_ptr<const MoveGroup::Goal> & goal,
    std::shared_ptr<MoveGroup::Result> & action_res);

  void startMoveExecutionCallback();
  void startMoveLookCallback();
  void preemptMoveCallback();
  void setMoveState(MoveGroupState state);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<MoveGroup>::SharedPtr move_action_server_;
  std::shared_ptr<GoalHandleMoveGroup> current_goal_handle_;
  moveit_msgs::action::MoveGroup::Feedback move_feedback_;

  MoveGroupState move_state_;
  std::atomic<bool> preempt_requested_;
  std::string name_;
};

}  // namespace move_group