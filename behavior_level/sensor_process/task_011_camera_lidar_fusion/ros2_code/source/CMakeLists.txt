cmake_minimum_required(VERSION 3.8)
project(task_011_camera_lidar_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# This is a reference-only package; the .cpp is not compiled into a real executable
# because it depends on autoware_msgs and UKF classes not available in vanilla Humble.
# The file is provided for static and runtime (source-level) testing.

ament_package()