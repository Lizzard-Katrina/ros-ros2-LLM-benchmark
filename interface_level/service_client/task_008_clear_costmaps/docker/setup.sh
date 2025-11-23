#!/bin/bash
# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Launch move_base and map server
roslaunch map_server map_server.launch map:=/workspace/catkin_ws/src/clear_costmaps_demo/maps/empty_map.yaml &
sleep 5

roslaunch move_base move_base.launch &
sleep 5

# Launch clear_costmaps client stub
rosrun clear_costmaps_demo clear_costmaps_client_stub.py
