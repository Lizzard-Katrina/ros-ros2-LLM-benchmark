cmake_minimum_required(VERSION 3.8)
project(task_003_ur5_red_box)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetJointStates.srv"
  DEPENDENCIES std_msgs
)

# Install Python scripts
install(PROGRAMS
  scripts/set_joint_states_service.py
  scripts/set_joint_states_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()