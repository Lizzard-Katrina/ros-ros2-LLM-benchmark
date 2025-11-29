# 010 Autonomous Vehicle Sensor Fusion Parameters

## Task Overview
This benchmark task evaluates the ability of a model to migrate ROS1-style
sensor fusion parameter loading for EKF nodes (robot_localization) to ROS2. 
Key migration challenge: replacing `rospy.get_param` calls with ROS2
`declare_parameter` and ensuring the EKF node initializes correctly.

## Directory Structure
010_sensor_fusion_params/
├── metadata.json
├── README.md
├── docker/
│   ├── Dockerfile
│   └── run.sh
└── ros1_code/
    └── version01_sensor_fusion.py

## Hole and Purpose
### version01_sensor_fusion.py
- **Hole:** Load EKF sensor fusion parameters from ROS1 parameter server
- **Reason:** Key migration point to ROS2 parameters
- **Expected outcome:** Model should replace `rospy.get_param` with ROS2 `declare_parameter` and ensure EKF node initializes correctly

## Docker Instructions
1. Build and run Docker image:
```bash
cd docker
chmod +x run.sh
./run.sh
```

2. Launch ROS1 environment with code ready for testing.

## Source

https://github.com/cra-ros-pkg/robot_localization
