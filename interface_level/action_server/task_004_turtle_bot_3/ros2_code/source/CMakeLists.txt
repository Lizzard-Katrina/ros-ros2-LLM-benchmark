cmake_minimum_required(VERSION 3.8)
project(task_004_turtle_bot_3)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/HeadPointing.action"
)

install(PROGRAMS
  scripts/head_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()