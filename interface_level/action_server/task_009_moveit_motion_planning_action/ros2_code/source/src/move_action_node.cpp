/*
 * Minimal ROS2 node that instantiates MoveGroupMoveAction
 * to verify compilation and provide a runnable executable for testing.
 */

#include "move_action_capability.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto move_action = std::make_shared<move_group::MoveGroupMoveAction>();
  move_action->initialize();

  RCLCPP_INFO(move_action->getNode()->get_logger(), "MoveGroupMoveAction node started");

  rclcpp::spin(move_action->getNode());
  rclcpp::shutdown();
  return 0;
}