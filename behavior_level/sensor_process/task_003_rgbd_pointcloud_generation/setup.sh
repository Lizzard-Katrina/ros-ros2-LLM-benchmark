#!/bin/bash

echo "Sourcing ROS1..."
source /opt/ros/noetic/setup.bash
source /root/ros1_ws/devel/setup.bash

echo "Container ready. Use the following commands:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source /root/ros1_ws/devel/setup.bash"
echo ""
echo "To run ROS1 launch file:"
echo "  roslaunch rgbd_task rgbd_to_pointcloud.launch"
bash
