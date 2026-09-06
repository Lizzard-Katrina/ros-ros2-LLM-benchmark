cmake_minimum_required(VERSION 3.5)
project(task_001_simple_map_navigation_2d)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps meshes rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()