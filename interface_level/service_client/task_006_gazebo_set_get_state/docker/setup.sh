#!/bin/bash
# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Launch Gazebo empty world
roslaunch gazebo_ros empty_world.launch &
sleep 5

# Launch set/get stub nodes
rosrun gazebo_state_demo set_model_state_stub.py &
sleep 2
rosrun gazebo_state_demo get_model_state_stub.py
