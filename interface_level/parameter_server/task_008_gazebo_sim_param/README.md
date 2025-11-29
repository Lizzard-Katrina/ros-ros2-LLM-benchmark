# 008 Gazebo Simulation Parameter Loading

## Task Overview
This benchmark task evaluates the ability of a model to migrate ROS1-style
parameter loading for Gazebo simulation nodes to ROS2. The key migration
challenge is replacing `rospy.get_param` calls with ROS2 `declare_parameter`
while ensuring controllers initialize with correct parameters.

## Directory Structure
008_gazebo_sim_param/
├── metadata.json
├── README.md
├── docker/
│   ├── Dockerfile
│   └── run.sh
└── ros1_code/
    └── version01_gazebo_params.py

## Holes and Purpose
### version01_gazebo_params.py
- **Hole:** Load all Gazebo simulation parameters from ROS1 parameter server
- **Reason:** Key migration point to ROS2 parameters
- **Expected outcome:** Model should replace `rospy.get_param` with ROS2 `declare_parameter` and ensure controllers initialize correctly

## Docker Instructions
1. Build and run Docker image:
```bash
cd docker
chmod +x run.sh
./run.sh
```
2. Launch ROS1 environment with code ready for testing

## Source
```https://github.com/ros-simulation/gazebo_ros_pkgs```
