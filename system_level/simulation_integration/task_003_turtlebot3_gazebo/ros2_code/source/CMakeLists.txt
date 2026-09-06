cmake_minimum_required(VERSION 3.8)
project(task_003_turtlebot3_gazebo)

find_package(ament_cmake REQUIRED)

# Install launch files
install(FILES
  turtlebot3_world.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(FILES
  turtlebot3_burger_cam.urdf
  turtlebot3_waffle.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()