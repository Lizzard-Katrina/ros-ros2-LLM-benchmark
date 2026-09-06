cmake_minimum_required(VERSION 3.8)
project(task_002_turtlebot3_patrol_action)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Patrol.action"
  DEPENDENCIES geometry_msgs
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/turtlebot3_patrol_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME turtlebot3_patrol_server
)

ament_package()