cmake_minimum_required(VERSION 3.8)
project(task_003_pr2_gripper_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_trajectory_action_node src/joint_trajectory_action.cpp)
ament_target_dependencies(joint_trajectory_action_node
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
  sensor_msgs
)

install(TARGETS
  joint_trajectory_action_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()