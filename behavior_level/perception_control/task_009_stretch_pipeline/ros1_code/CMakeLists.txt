cmake_minimum_required(VERSION 3.0.2)
project(stretch_pipeline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stretch_main src/main_launch.cpp
  src/perception_node.cpp
  src/mapping_node.cpp
  src/planner_node.cpp
  src/controller_node.cpp
)
target_link_libraries(stretch_main ${catkin_LIBRARIES})
