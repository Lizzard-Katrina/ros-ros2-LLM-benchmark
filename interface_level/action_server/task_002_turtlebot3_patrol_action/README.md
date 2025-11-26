# Task 002: TurtleBot3 Patrol Action

## Purpose
Simulate a TurtleBot3 patrol action server. Client sends a patrol goal containing multiple waypoints; server sequentially updates feedback and returns a result.

## Directory
- `ros1_code/` → action server stub + ROS package
- `docker/` → builds ROS workspace and environment
- `tests/` → optional, for verifying server feedback/results
- `README.md`, `metadata.md` → documentation

## Build & Run
1. Build Docker image:
```bash
cd docker
docker build -t turtlebot3_patrol_action .```

2. Run container:

```docker run -it --rm turtlebot3_patrol_action /bin/bash```


3. Inside container, source workspace:

```source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
rosrun turtlebot3_patrol_action turtlebot3_patrol_action_server_stub.py```
