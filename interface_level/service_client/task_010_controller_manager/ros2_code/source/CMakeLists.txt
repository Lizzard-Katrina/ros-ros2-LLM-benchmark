cmake_minimum_required(VERSION 3.8)
project(task_010_controller_manager)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerState.msg"
  "msg/HardwareInterfaceResources.msg"
  "srv/ListControllerTypes.srv"
  "srv/ListControllers.srv"
  "srv/LoadController.srv"
  "srv/UnloadController.srv"
  "srv/SwitchController.srv"
  "srv/ReloadControllerLibraries.srv"
  DEPENDENCIES std_msgs
)

# Install Python scripts
install(PROGRAMS
  controller_manager_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()