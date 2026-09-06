/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 *   Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <odom_tracker/odom_tracker.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cl_move_base_z::odom_tracker::OdomTracker>("odom", "odom");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}