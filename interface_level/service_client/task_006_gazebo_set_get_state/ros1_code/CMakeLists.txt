cmake_minimum_required(VERSION 3.0.2)
project(gazebo_state_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_msgs
  geometry_msgs
  ________  # TODO: additional dependencies
)

catkin_package()

catkin_install_python(PROGRAMS
  set_model_state_stub.py
  get_model_state_stub.py
  ________  # TODO: extra scripts if any
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
