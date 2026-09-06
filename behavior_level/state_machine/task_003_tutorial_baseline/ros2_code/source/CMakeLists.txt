cmake_minimum_required(VERSION 3.8)
project(task_003_tutorial_baseline)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModelState.msg"
  "srv/SetModelState.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python package — call ONCE only
ament_python_install_package(task_003_tutorial_baseline_py
  PACKAGE_DIR task_003_tutorial_baseline_py
)

# Install scripts
install(PROGRAMS
  scripts/sim_monitor_state
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()