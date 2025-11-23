cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_service_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  __________
)

catkin_package()

catkin_install_python(PROGRAMS
  __________
  __________
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
