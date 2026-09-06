cmake_minimum_required(VERSION 3.5)
project(task_002_husky_sim_navigation)

find_package(ament_cmake REQUIRED)

# Install config, launch, and maps directories to share space
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()