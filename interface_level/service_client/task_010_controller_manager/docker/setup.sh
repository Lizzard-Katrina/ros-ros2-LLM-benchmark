#!/bin/bash
# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Launch controller manager (example robot)
roslaunch controller_manager controller_manager.launch &
sleep 10

# Launch client stub
rosrun controller_manager_demo controller_manager_client_stub.py
