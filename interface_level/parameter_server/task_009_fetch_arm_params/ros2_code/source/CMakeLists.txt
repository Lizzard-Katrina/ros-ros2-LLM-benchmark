cmake_minimum_required(VERSION 3.8)
project(task_009_fetch_arm_params)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# We do NOT attempt to build the full IKFast plugin (requires MoveIt2, IKFast solver, etc.)
# The source file is provided for static and runtime inspection tests.
# Install the source file so tests can find it.
install(FILES
  fetch_arm_ikfast_moveit_plugin.cpp
  DESTINATION share/${PROJECT_NAME}
)

ament_package()