cmake_minimum_required(VERSION 3.8)
project(task_002_custom_msg_basic)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
)

# Install Python nodes
install(PROGRAMS
  publisher_node.py
  subscriber_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()