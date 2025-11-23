#!/bin/bash
# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash

# Launch robot_state_publisher and move_group
roslaunch moveit_commander_demo move_group.launch &
sleep 10

# Launch planning scene client stub
rosrun moveit_planning_scene_demo apply_planning_scene_client_stub.py
