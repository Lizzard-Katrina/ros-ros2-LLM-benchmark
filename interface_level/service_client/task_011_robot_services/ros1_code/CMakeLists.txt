cmake_minimum_required(VERSION 3.0.2)
project(robot_service_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ________  # TODO: robot-specific service dependencies
)

catkin_package()

catkin_install_python(PROGRAMS
  robot_service_client_stub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
