# 005 Laser Scan Filter Parameters

## Task Overview
This benchmark task evaluates the ability of a model to migrate ROS1-style
laser scan filter parameter loading to ROS2 node parameters. The key migration
challenge is replacing `rospy.get_param` calls with `declare_parameter()` while
maintaining correct behavior for dynamic reconfigure if applicable.

## Directory Structure
005_laser_scan_filter_params/
├── metadata.json
├── README.md
├── docker/
│   ├── Dockerfile
│   └── run.sh
└── ros1_code/
    └── version01_load_filter_params.py

## Holes and Purpose
### version01_load_filter_params.py
- **Hole:** Loading laser filter parameters from ROS1 parameter server
- **Reason:** Key migration point for ROS2 parameters
- **Expected outcome:** Model should replace `rospy.get_param` with ROS2 `declare_parameter` and ensure filter parameters are available for node initialization

## Docker Instructions
1. Build and run Docker image:
```bash
cd docker
chmod +x run.sh
./run.sh
```

2. Launches ROS1 environment with code ready for testing

## Source

```https://github.com/ros-perception/laser_filters
```

## Notes
Only ROS1 code is provided.

The file contains one TODO for model to complete.
