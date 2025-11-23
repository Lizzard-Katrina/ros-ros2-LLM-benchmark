# Task 002: Create & Use Custom ROS Service (.srv)

## Overview
This task extends the simple ROS service example by requiring a **custom .srv interface**.  
The user must define `AddThreeInts.srv`, generate message/service headers using `catkin_make`, and implement both a service and a client.

ROS1 skeleton code is provided with TODO markers.  
ROS2 implementation is NOT provided; expected outcome guides generation.

---

## Directory Structure
ros1_code/ # ROS1 skeleton with custom srv directory
├── srv/
│ └── AddThreeInts.srv
├── ros1_server.py
└── ros1_client.py
docker/ # Reused ROS1+ROS2 dual-environment container
tests/
metadata.json

---

## Expected Outcome
- `.srv` file is correctly placed in `srv/` and registered in CMakeLists.txt / package.xml (for ROS1 or ROS2).
- `catkin_make` successfully generates Python service code.
- ROS1 service server starts without errors.
- ROS1 client calls server and receives correct sum result.
- ROS2 implementation should mirror ROS1 behavior.

---

## Docker Instructions
Use the same Docker environment as task_001.

Build:

```bash
cd docker
docker build -t ros_task002_custom_srv .

Run:

docker run -it --rm -v $(pwd)/..:/workspace ros_task002_custom_srv ./setup.sh
Inside container (ROS1 test):

cd /workspace/ros1_code
python3 ros1_server.py
python3 ros1_client.py ```

Note:
This task tests correct custom SRV creation and build configuration.
No ROS2 code is provided; expected behavior guides translation.

