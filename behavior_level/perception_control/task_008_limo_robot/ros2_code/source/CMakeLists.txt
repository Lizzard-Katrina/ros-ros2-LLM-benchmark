cmake_minimum_required(VERSION 3.8)
project(task_008_limo_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(limo_base_node
  src/limo_driver.cpp
  src/serial_port.cpp
  src/limo_base_node.cpp
)

ament_target_dependencies(limo_base_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  limo_base_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()