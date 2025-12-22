# Task 003: Multi-node ROS1 Perception → Control

## Description
Multi-node ROS1 example where sensor data is processed and control commands are published.  
Flow: `/scan` → processing → `/cmd_vel`

## Expected outcome for each TODO

1. **constructor_init.cpp**:
   - Subscribers and publishers initialized
   - Processing system parameters initialized
   - Node compiles successfully

2. **sensor_processing.cpp**:
   - LaserScan data processed
   - Features or measurements extracted
   - `processed_value_` updated
   - `data_ready_` set true

3. **command_publisher.cpp**:
   - `processed_value_` converted to `cmd_vel`
   - Robot moves according to sensor input
   - Data_ready flag handled properly

## Validation
- Each TODO can be tested independently
- Full system: robot reacts to obstacles in simulated or real LaserScan environment
