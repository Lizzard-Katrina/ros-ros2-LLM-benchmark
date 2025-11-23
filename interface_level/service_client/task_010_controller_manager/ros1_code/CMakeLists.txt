cmake_minimum_required(VERSION 3.0.2)
project(controller_manager_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  controller_manager_msgs
  ________  # TODO: extra dependencies if needed
)

catkin_package()

catkin_install_python(PROGRAMS
  controller_manager_client_stub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
