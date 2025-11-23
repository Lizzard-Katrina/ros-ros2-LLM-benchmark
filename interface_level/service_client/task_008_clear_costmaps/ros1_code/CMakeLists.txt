cmake_minimum_required(VERSION 3.0.2)
project(clear_costmaps_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  ________  # TODO: add additional dependencies if needed
)

catkin_package()

catkin_install_python(PROGRAMS
  clear_costmaps_client_stub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
