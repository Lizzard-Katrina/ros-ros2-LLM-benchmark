cmake_minimum_required(VERSION 3.8)
project(task_002_custom_srv)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddThreeInts.srv"
)

install(PROGRAMS
  ros_server.py
  ros_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()