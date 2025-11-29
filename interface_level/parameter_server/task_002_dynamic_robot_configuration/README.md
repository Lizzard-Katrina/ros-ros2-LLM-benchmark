# 002 Dynamic Robot Configuration (YAML-based)

## Task Overview
This benchmark task evaluates the ability of a model to translate ROS1-style
YAML-based parameter loading into ROS2 parameters. The key migration challenge is
to replace `rosparam load` or `rospy.get_param` with ROS2 node-declared parameters.

## Directory Structure
002_dynamic_robot_configuration/
├── metadata.json
├── README.md
├── docker/
│   ├── Dockerfile
│   └── run.sh
└── ros1_code/
    └── version01_load_yaml.py

## Holes and Purpose
### version01_load_yaml.py
- **Hole:** Loading YAML configuration into ROS1 parameter server
- **Reason:** This is the main challenge for migrating to ROS2 parameters
- **Expected outcome:** Model should convert to ROS2 `declare_parameter` and YAML loading

## Docker Instructions
1. Build and run Docker image:
```bash
cd docker
chmod +x run.sh
./run.sh```

2. This launches a ROS1 environment with the code ready for testing.
