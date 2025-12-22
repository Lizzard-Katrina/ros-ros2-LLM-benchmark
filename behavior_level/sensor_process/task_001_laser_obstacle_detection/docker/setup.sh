#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /root/ros1_ws
catkin_make

echo "ROS1 workspace built."
