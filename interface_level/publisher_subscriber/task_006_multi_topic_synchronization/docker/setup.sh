#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash

cd /root/ros1_ws
rosdep install --from-paths src -r -y
catkin_make

cd /root/ros2_ws
rosdep install --from-paths src -r -y
colcon build
