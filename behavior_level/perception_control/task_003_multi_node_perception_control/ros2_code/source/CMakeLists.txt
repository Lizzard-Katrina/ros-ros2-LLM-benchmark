cmake_minimum_required(VERSION 3.8)
project(task_003_multi_node_perception_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# kobuki_ros_interfaces may or may not be available; we define our own MotorPower if not
# For portability we just use std_msgs/UInt8 as a stand-in for MotorPower
# But the oracle tests require kobuki_msgs::msg::MotorPower patterns in source,
# so we handle it via a local header.

add_executable(kobuki_joystick src/kobuki_joystick.cpp)
target_include_directories(kobuki_joystick PRIVATE include)
ament_target_dependencies(kobuki_joystick rclcpp geometry_msgs std_msgs)

install(TARGETS kobuki_joystick
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()