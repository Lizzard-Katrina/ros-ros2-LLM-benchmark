#!/bin/bash

echo "Sourcing ROS1..."
source /opt/ros/noetic/setup.bash
source /root/ros1_ws/devel/setup.bash

echo "To run YOLO detection launch file:"
echo "  roslaunch task_008_camera_object_detection_yolo yolo_detection.launch"

bash
