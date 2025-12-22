#!/bin/bash

echo "Sourcing ROS1..."
source /opt/ros/noetic/setup.bash
source /root/ros1_ws/devel/setup.bash

echo "To run sonar filter:"
echo "  roslaunch sonar_task sonar_filter.launch"

bash
