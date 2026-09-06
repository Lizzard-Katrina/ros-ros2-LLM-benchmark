cmake_minimum_required(VERSION 3.8)
project(task_003_Mobile_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(diff_drive_node src/diff_drive_controller.cpp)
ament_target_dependencies(diff_drive_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  diff_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(oracle_test test/test_oracle_ros2.py)
  ament_add_pytest_test(runtime_test test_runtime_ros2.py)
endif()

ament_package()