cmake_minimum_required(VERSION 3.0.2)
project(moveit_planning_scene_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_msgs
  ________  # TODO: add extra dependencies if needed
)

catkin_package()

catkin_install_python(PROGRAMS
  apply_planning_scene_client_stub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
