cmake_minimum_required(VERSION 3.0.2)
project(allegro_hand_v5_ros1_task)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  gazebo_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_imu_plugin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(gazebo_imu_plugin
  src/gazebo_imu_plugin.cpp
)

target_link_libraries(gazebo_imu_plugin
  ${catkin_LIBRARIES}
)
