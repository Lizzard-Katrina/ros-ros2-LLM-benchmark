#!/bin/bash
# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Launch map server + move_base (example empty map)
roslaunch map_server map_server.launch map:=/workspace/catkin_ws/src/navigation_make_plan_demo/maps/empty_map.yaml &
sleep 5

roslaunch move_base move_base.launch &
sleep 5

# Launch make_plan stub node
rosrun navigation_make_plan_demo make_plan_client_stub.py
