cmake_minimum_required(VERSION 3.8)
project(task_008_gazebo_sim_param)

find_package(ament_cmake REQUIRED)

# This package provides a migrated C++ source file for analysis.
# The .cpp file depends on Gazebo/Ignition headers not available in
# a pure ROS 2 Humble environment, so it is installed as a data file
# rather than compiled.

install(FILES gazebo_ros_camera_utils.cpp
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()