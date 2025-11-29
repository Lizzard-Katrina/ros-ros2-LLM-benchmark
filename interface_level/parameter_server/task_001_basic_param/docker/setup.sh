#!/bin/bash

echo "Setting up ROS1 & ROS2..."

source /opt/ros/noetic/setup.sh
source /ros1_ws/devel/setup.sh

source /opt/ros/foxy/setup.sh
source /ros2_ws/install/setup.sh

echo "Available ROS1 test executables:"
echo "  version_01_read_param"
echo "  version_02_log_param"
echo "  version_03_update_param"
