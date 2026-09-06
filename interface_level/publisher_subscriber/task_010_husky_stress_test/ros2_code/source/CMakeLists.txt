cmake_minimum_required(VERSION 3.8)
project(task_010_husky_stress_test)

find_package(ament_cmake REQUIRED)

# Install the data files
install(FILES
  husky_empty_world.launch
  husky.gazebo.xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()