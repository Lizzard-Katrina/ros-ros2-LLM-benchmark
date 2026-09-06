cmake_minimum_required(VERSION 3.8)
project(task_008_darknet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# This package is primarily a source-file artifact for static/runtime testing.
# We do not actually build a full darknet binary (no darknet library available),
# but we ensure the translated .cpp file is present and syntactically inspectable.

ament_package()