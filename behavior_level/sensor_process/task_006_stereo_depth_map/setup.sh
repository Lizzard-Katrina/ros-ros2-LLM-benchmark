#!/bin/bash

echo "Sourcing ROS1..."
source /opt/ros/noetic/setup.bash
source /root/ros1_ws/devel/setup.bash

echo "Run stereo depth processor:"
echo "  roslaunch stereo_task stereo_depth.launch"

bash
