cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectsStamped.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

install(PROGRAMS
  scripts/object_detector.py
  scripts/pick_and_place_state_machine.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()