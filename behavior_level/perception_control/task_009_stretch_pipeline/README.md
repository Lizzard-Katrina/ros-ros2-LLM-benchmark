# Task 009 — Hello Robot Stretch Mobile Manipulator Pipeline

## Overview
Perception → Mapping → Planner (arm + base) → Controller → Actuators.

This ROS1 scaffold mirrors the Stretch ROS architecture but keeps core algorithmic parts as TODOs for the benchmark:
- `ros1_code/src/perception_node.cpp` — detect objects (TODO)
- `ros1_code/src/mapping_node.cpp` — maintain local map (TODO)
- `ros1_code/src/planner_node.cpp` — compute arm/base plan (TODO)
- `ros1_code/src/controller_node.cpp` — execute plan (TODO)

## How to use
1. Create package folder under `catkin_ws/src/` and copy `ros1_code` contents into that package.
2. Build with `catkin_make` (or `catkin build`).
3. Run:
   ```bash
   roslaunch stretch_pipeline stretch_pipeline.launch```
4. Provide mock /camera/* topics or run with real Stretch stack.
## Expected outcome per TODO

Perception publishes /detected_objects messages (object IDs and poses).

Mapping publishes /local_map (simple occupancy grid or object table).

Planner publishes /planned_path (arm + base trajectory).

Controller publishes /stretch_controller/cmd and /arm_controller/command.
