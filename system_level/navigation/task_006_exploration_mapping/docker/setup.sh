#!/bin/bash
source /opt/ros/noetic/setup.bash
cd /catkin_ws
catkin_make
source devel/setup.bash
exec "$@"
