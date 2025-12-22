#!/bin/bash

echo "Sourcing ROS1 Noetic..."
source /opt/ros/noetic/setup.bash
source /root/ros1_ws/devel/setup.bash

echo "ROS1 workspace ready."
echo "To run the EKF node:"
echo "  roslaunch imu_task imu_to_odom.launch"

bash
