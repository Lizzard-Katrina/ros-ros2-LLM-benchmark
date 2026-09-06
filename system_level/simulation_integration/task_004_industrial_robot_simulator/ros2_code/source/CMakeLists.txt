cmake_minimum_required(VERSION 3.8)
project(task_004_industrial_robot_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# We don't actually build the .cpp files as executables since industrial_robot_client
# headers are not available. The files are provided for static and runtime analysis.
# The launch file is installed for testing purposes.

install(
  FILES robot_interface_simulator.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  FILES generic_robot_state_node.cpp joint_trajectory_interface.cpp
  DESTINATION share/${PROJECT_NAME}/src
)

ament_package()