# Allegro Hand V5 ROS1 Task

## Overview
This task provides a skeleton scaffold for the Allegro Hand V5 ROS1 integration. The goal is to implement a Gazebo IMU plugin and MAVConn message buffer in a ROS1-compatible package.

## Structure
- `include/` - header files (e.g., `msgbuffer.h`)
- `src/` - source files (e.g., `gazebo_imu_plugin.cpp`)
- `metadata.json` - task metadata
- `README.md` - task description

## Instructions
1. Fill in the TODO in `addNoise()` in `gazebo_imu_plugin.cpp`.
2. Make sure the package compiles with `catkin_make`.
3. Test the Gazebo IMU plugin in simulation.

## References
- Allegro Hand ROS: [GitHub](https://github.com/Wonikrobotics-git/allegro_hand_ros_v5)
- Gazebo Plugins: http://gazebosim.org/tutorials

