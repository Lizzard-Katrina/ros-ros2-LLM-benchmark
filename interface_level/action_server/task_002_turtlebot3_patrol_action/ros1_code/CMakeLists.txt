cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_patrol_action)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  turtlebot3_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 如果有 action 文件可以生成，这里要加 action 生成指令
# 但是因为是 stub，暂时不需要
