# 003 Mobile Robot Speed Control

## Task Overview
This benchmark task evaluates the ability of a model to migrate ROS1-style
speed parameter loading to ROS2 node parameters. The key challenge is
correctly loading and using speed limits from the parameter server.

## Directory Structure
003_mobile_robot_speed_control/
├── metadata.json
├── README.md
├── docker/
│   ├── Dockerfile
│   └── run.sh
└── ros1_code/
    └── version01_load_params.py

## Holes and Purpose
### version01_load_params.py
- **Hole:** Loading speed limit parameters from ROS1 parameter server
- **Reason:** This is the key migration point to ROS2 parameters
- **Expected outcome:** Model should replace `rospy.get_param` with `declare_parameter()` and load YAML speed limits correctly

## Docker Instructions
1. Build and run Docker image:
```bash
cd docker
chmod +x run.sh
./run.sh
```

2. Launches ROS1 environment with code ready for testing.

## Source

1. ```https://wiki.ros.org/diff_drive_controller```
2. ```https://github.com/ros-controls/ros_controllers```

## Notes

1. Only ROS1 code is provided.

2. The file contains one TODO for model completion.

