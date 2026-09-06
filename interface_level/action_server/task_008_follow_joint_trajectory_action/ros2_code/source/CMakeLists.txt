cmake_minimum_required(VERSION 3.5)
project(task_008_follow_joint_trajectory_action)

find_package(ament_cmake REQUIRED)

# This package provides a .cpp file for static analysis / oracle tests.
# No compiled targets are needed since the oracle tests do regex-based
# source inspection and the runtime test validates the source content
# programmatically.

install(
  FILES trajectory_planner_ros.cpp
  DESTINATION share/${PROJECT_NAME}
)

ament_package()