# 004 TurtleBot3 Model Parameters

## Task Overview
This benchmark task evaluates the ability of a model to migrate ROS1-style
robot model parameter loading into ROS2 node parameters. Key migration challenge
is replacing `rospy.get_param` calls with ROS2 `declare_parameter()` calls
so that all nodes correctly initialize with model parameters.

## Directory Structure
004_turtlebot3_model_params/
├── metadata.json
├── README.md
├── docker/
│   ├── Dockerfile
│   └── run.sh
└── ros1_code/
    └── version01_load_model_params.py

## Holes and Purpose
### version01_load_model_params.py
- **Hole:** Loading TurtleBot3 model parameters from ROS1 parameter server
- **Reason:** This is the key migration point for ROS2
- **Expected outcome:** Model should replace `rospy.get_param` with ROS2 `declare_parameter` and correctly load model params (robot_type, wheel_diameter, wheel_base)

## Docker Instructions
1. Build and run Docker image:
```bash
cd docker
chmod +x run.sh
./run.sh
```

2. Launches a ROS1 environment with code ready for testing.

3. Source

```https://github.com/ROBOTIS-GIT/turtlebot3```
4. Notes
Only ROS1 code is provided.

The file contains one TODO for model to complete.


