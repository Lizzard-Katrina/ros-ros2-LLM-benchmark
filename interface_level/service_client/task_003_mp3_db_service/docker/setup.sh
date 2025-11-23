#!/bin/bash

echo "Sourcing ROS1 Noetic..."
source /opt/ros/noetic/setup.bash
source $ROS1_WS/devel/setup.bash

echo "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash
source $ROS2_WS/install/setup.bash

echo "ROS1 & ROS2 environments activated."
exec "$@"
