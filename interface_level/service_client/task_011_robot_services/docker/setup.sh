#!/bin/bash
# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Launch robot simulation
# Uncomment the one you want to test:
# roslaunch fetch_gazebo_demo fetch_gazebo_demo.launch &
# roslaunch pr2_gazebo pr2_gazebo.launch &

sleep 15  # wait for simulation to be ready

# Launch client stub
rosrun robot_service_demo robot_service_client_stub.py
