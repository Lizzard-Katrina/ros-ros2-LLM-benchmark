#!/bin/bash
# 激活 ROS 环境
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
exec "$@"
