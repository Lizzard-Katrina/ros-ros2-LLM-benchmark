#!/bin/bash
set -e
mkdir -p /root/ws/src
# copy code from mounted folder (assume /workspace mounted)
cp -r /workspace/ros1_code /root/ws/src/robot_calibration_action

cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# Launch the server (user will fill stub or AI-generated server)
rosrun robot_calibration_action calibration_action_server_stub.py &
sleep 2
