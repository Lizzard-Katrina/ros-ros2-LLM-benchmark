cmake_minimum_required(VERSION 3.8)
project(task_009_moveit_motion_planning_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(move_action_capability SHARED
  src/move_action_capability.cpp
)
target_include_directories(move_action_capability PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(move_action_capability
  rclcpp
  rclcpp_action
  moveit_msgs
  std_msgs
)

install(TARGETS move_action_capability
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES src/move_action_capability.cpp
  DESTINATION share/${PROJECT_NAME}
)

ament_package()