cmake_minimum_required(VERSION 3.0.2)
project(robot_calibration_action)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  std_msgs
)

add_action_files(
  DIRECTORY action
  FILES RobotCalibration.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
