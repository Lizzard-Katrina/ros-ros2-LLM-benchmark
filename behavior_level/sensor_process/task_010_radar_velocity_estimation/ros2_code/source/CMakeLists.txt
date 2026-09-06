cmake_minimum_required(VERSION 3.8)
project(task_010_radar_velocity_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# This is a reference-only package; the main .cpp is not compiled into a binary
# because it depends on external libraries (estimator_, profiler, pcl, etc.)
# that are not available. We install the source file for static/runtime testing.

install(
  FILES radar_ego_velocity_estimator_ros.cpp
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()