cmake_minimum_required(VERSION 2.8.3)
project(simple_pub_sub)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/simple_pub.py
  scripts/simple_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
