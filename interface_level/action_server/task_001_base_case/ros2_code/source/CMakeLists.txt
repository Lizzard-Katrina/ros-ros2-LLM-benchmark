cmake_minimum_required(VERSION 3.8)
project(task_001_base_case)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RobotCalibration.action"
)

# Install Python node script
install(PROGRAMS
  calibration_action_server_stub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()