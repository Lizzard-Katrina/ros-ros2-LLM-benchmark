# Task 007: Autonomous Navigation

## Flow
LiDAR + Camera → Sensor Fusion → Path Planning → Motion Control

Example based on Turtlebot3 (SLAM, navigation, simulation).

## Expected outcomes per TODO

### constructor_init.cpp
- Parameters loaded (speed limit, radius, etc.)
- Node compiles and runs

### sensor_fusion.cpp
- Fusion of LiDAR + camera into local costmap/obstacle representation

### path_planning.cpp
- Planned local path published on `/planned_path`

### motion_controller.cpp
- Robot moves according to planned path
- `/cmd_vel` publishes valid linear/angular commands

## Test
Run in Turtlebot3 Gazebo simulation using turtlebot3_simulations.
