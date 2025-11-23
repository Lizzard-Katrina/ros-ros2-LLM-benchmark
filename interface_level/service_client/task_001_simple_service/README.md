# Task 001: Simple Python Service & Client

## Description
- Client asks server to sum two numbers.
- ROS1 skeleton code is provided; ROS2 implementation should be completed by AI/user.

## Directory Structure
ros1_code/ # ROS1 skeleton Python files
tests/ # Optional testing scripts
docker/ # Dockerfile and setup.sh
metadata.json # Task metadata


## Expected Outcome
- Service node starts successfully.
- Client call returns correct sum (e.g., 2 + 3 = 5).
- ROS2 scripts run without errors.

## Docker Instructions
1. Build Docker image:
   ```bash
   cd docker
   docker build -t ros1_ros2_service_task001.

2. Run container and activate ROS environment:
docker run -it --rm -v $(pwd)/..:/workspace ros1_ros2_service_task001 ./setup.sh

3. inside container:
cd /workspace/ros1_code
python3 ros1_server.py
python3 ros1_client.py

