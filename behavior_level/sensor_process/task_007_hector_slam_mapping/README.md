# Task 007 – iDAR → 3D Mapping (Hector SLAM)

## Overview
This task benchmarks a model's ability to convert ROS1 Hector SLAM–related code to ROS2.
The ROS1 node listens to /scan, processes the SLAM output, and subscribes to /map and /pose.

Source:
https://github.com/tu-darmstadt-ros-pkg/hector_slam

## ROS1 Code Description
Two ROS1 Python files are included:
- scan_subscriber.py: subscribes to /scan
- hector_mapping_node.py: subscribes to /map and /pose

Each file contains exactly one TODO block where the LLM must fill missing ROS2 logic during conversion.

## Expected ROS2 Outcome
The converted ROS2 version should:
- Subscribe to /scan (sensor_msgs/msg/LaserScan)
- Launch /hector_slam components (if available or replicated)
- Subscribe to /map (nav_msgs/msg/OccupancyGrid)
- Subscribe to /pose (geometry_msgs/msg/PoseStamped)
- Follow ROS2 composition and launch standards

## Build Docker
```
cd behavior_level/sensor_process/task_007_hector_slam_mapping
docker build -t task007 .
```

## Run

```
docker run -it task007
```

This loads a ROS1 environment with Hector SLAM preinstalled.
