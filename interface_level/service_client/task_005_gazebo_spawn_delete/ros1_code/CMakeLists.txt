cmake_minimum_required(VERSION 3.0.2)
project(gazebo_service_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_msgs
  geometry_msgs
  ________  # TODO: any additional dependencies
)

catkin_package()

catkin_install_python(PROGRAMS
  spawn_model_stub.py
  delete_model_stub.py
  ________  # TODO: extra scripts if any
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
