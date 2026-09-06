cmake_minimum_required(VERSION 3.8)
project(task_001_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# Also install root-level files for oracle test compatibility
install(
  FILES macroed.urdf.xacro display.launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()