cmake_minimum_required(VERSION 3.8)
project(task_013_slam_mapping_params)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(slam_gmapping_node src/slam_gmapping.cpp)
ament_target_dependencies(slam_gmapping_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS slam_gmapping_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()