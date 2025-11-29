# 009 Fetch Robot Arm Controller Parameters

## Task Overview
This benchmark task evaluates the ability of a model to migrate ROS1-style
controller parameter loading for the Fetch robot arm to ROS2. Key migration
challenge: replacing `rospy.get_param` calls with ROS2 `declare_parameter`
and ensuring the arm controllers initialize correctly.

## Directory Structure
009_fetch_arm_params/
├── metadata.json
├── README.md
├── docker/
│   ├── Dockerfile
│   └── run.sh
└── ros1_code/
    └── version01_fetch_arm_params.py

## Holes and Purpose
### version01_fetch_arm_params.py
- **Hole:** Load all Fetch arm controller parameters from ROS1 parameter server
- **Reason:** Key migration point to ROS2 parameters
- **Expected outcome:** Model should replace `rospy.get_param` with ROS2 `declare_parameter` and ensure arm controllers initialize correctly

## Docker Instructions
1. Build and run Docker image:
```bash
cd docker
chmod +x run.sh
./run.sh
```

2. Launch ROS1 environment with code ready for testing.

## source

```https://github.com/ZebraDevs/fetch_ros```

