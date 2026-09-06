cmake_minimum_required(VERSION 3.8)
project(task_007_navigation_sensor_failure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# This package only provides source files for migration validation.
# No executables are built because the full turtlesim depends on Qt, turtle.hpp, etc.
# The files are validated via static oracle tests and a runtime test that checks file content.

ament_package()