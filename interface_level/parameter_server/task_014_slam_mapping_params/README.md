# Task 014 — SLAM Mapping Config Parameters
**Category:** Interface Level / Parameter Server  
**Source:** https://github.com/ros-perception/slam_gmapping  
**ROS Version:** ROS1 (Noetic)

This task focuses on how the SLAM gmapping node loads configuration parameters from ROS parameter server (e.g., map update rate, max range, odom frame).  
The benchmark removes some of these parameter-loading lines from the real node initialization logic.

Students must restore missing ROS parameter calls and ensure the node logs or uses the parameters correctly.

## Versions
- **v01 (Easy):** One missing parameter.
- **v02 (Medium):** Multiple missing parameters.
- **v03 (High):** Grouped parameters removed; core logic depending on them removed.

## Purpose
To validate whether a system can recover:
1. Correct `rospy.get_param` usage  
2. Naming conventions (`~param_name`)  
3. Parameter-dependent SLAM initialization and logging

## Expected Outcome
- Node runs without `KeyError` for missing parameters.  
- Node logs correct parameter values during mapping.  
- Feedback values and mapping rate reflect restored parameter logic in all versions.
