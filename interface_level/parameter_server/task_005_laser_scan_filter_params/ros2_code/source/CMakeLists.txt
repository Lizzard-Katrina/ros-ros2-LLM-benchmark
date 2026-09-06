cmake_minimum_required(VERSION 3.8)
project(task_005_laser_scan_filter_params)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(filters REQUIRED)

add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
ament_target_dependencies(generic_laser_filter_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  filters
)

install(TARGETS
  generic_laser_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()