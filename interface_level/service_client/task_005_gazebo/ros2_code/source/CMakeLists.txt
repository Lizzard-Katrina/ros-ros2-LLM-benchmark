cmake_minimum_required(VERSION 3.8)
project(task_005_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpawnEntity.srv"
  "srv/SetModelConfiguration.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# Install the pure-python library module (gazebo_interface.py) as a
# separate package name so it doesn't clash with the rosidl-generated one.
# We use a differently-named directory: task_005_gazebo_py
ament_python_install_package(task_005_gazebo_py
  PACKAGE_DIR task_005_gazebo_py
)

install(PROGRAMS
  scripts/gazebo_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_interface_node
)

ament_package()