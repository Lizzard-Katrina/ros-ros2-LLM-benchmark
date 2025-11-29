# 006 Dynamic Parameter Change via rqt_reconfigure

## Task Overview
This benchmark task evaluates the ability of a model to migrate ROS1-style
dynamic parameter reconfigure nodes to ROS2 parameters. The key migration
challenge is replacing `dynamic_reconfigure.Server` callbacks with ROS2
parameter events so that node logic updates in real-time.

## Directory Structure
006_dynamic_param_rqt_reconfigure/
├── metadata.json
├── README.md
├── docker/
│   ├── Dockerfile
│   └── run.sh
└── ros1_code/
    └── version01_dynamic_reconfigure.py

## Holes and Purpose
### version01_dynamic_reconfigure.py
- **Hole:** Setup dynamic parameter reconfigure callbacks
- **Reason:** This is the key migration point to ROS2 parameter events
- **Expected outcome:** Model should replace dynamic_reconfigure usage with ROS2 parameter callbacks and update node logic in real-time

## Docker Instructions
1. Build and run Docker image:
```bash
cd docker
chmod +x run.sh
./run.sh
```

2. Launches a ROS1 environment with code ready for testing.

## source

https://wiki.ros.org/rqt_reconfigure

https://github.com/ros-visualization/rqt_reconfigure

## notes

Only ROS1 code is provided.

The file contains one TODO for model to complete.
