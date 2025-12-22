# Task 008: Robot Perception System in Simulation + Virtual Control

## Flow
Gazebo simulated sensors → perception processing → controller → robot actuators in simulation.

## Components

### constructor_init.cpp
Loads simulation control parameters (speed, thresholds).

### perception.cpp
Processes simulated camera + LiDAR data:
- CV image processing
- Basic obstacle detection
- LiDAR-based range checking

### virtual_controller.cpp
Uses the perception output to:
- Avoid obstacles
- Move forward when clear
- Publish /cmd_vel to Gazebo robot

## Test
Recommended:
