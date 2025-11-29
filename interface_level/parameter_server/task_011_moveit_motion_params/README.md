# Task 011: MoveIt Motion Planning Parameters

## Description
This task focuses on ROS1 MoveIt tutorials for motion planning. Users must complete the TODOs to correctly load parameters from the ROS1 param server or YAML configuration.


## ROS1 Code TODOs

| File | TODO Description | Expected Outcome |
|------|-----------------|----------------|
| motion_planning_param_v01.py | Load planner parameters from param server | Node reads params and executes motion plan using correct planner ID |
| motion_planning_param_v02.py | Load velocity and acceleration scaling from param server | Motion executes with correct velocity/acceleration scaling |
| motion_planning_param_v03.py | Load YAML planner configuration | Motion executes using loaded YAML planner parameters |

## Docker Setup

### Build Image
```bash
cd docker
chmod +x setup.sh
./setup.sh

### Run container
docker run -it --rm -v ~/benchmark:/root/benchmark moveit_motion_params:latest


