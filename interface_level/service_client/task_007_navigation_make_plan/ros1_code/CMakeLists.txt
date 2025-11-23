cmake_minimum_required(VERSION 3.0.2)
project(navigation_make_plan_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  move_base_msgs
  geometry_msgs
  nav_msgs
  ________  # TODO: additional dependencies
)

catkin_package()

catkin_install_python(PROGRAMS
  make_plan_client_stub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
