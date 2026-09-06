cmake_minimum_required(VERSION 3.8)
project(task_012_move_arm_cartesian_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# The main source file (move_group_interface.cpp) requires moveit packages
# which may not be available at build time. We install it as a data/source file
# so it can be compiled in environments where MoveIt is available.
install(FILES
  move_group_interface.cpp
  DESTINATION share/${PROJECT_NAME}/src
)

ament_package()