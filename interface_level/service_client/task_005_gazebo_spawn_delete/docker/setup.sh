#!/bin/bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# 启动 Gazebo 空世界
roslaunch gazebo_ros empty_world.launch &
sleep 5

# 启动 spawn / delete 节点
rosrun gazebo_service_demo spawn_model_stub.py &
sleep 2
rosrun gazebo_service_demo delete_model_stub.py
