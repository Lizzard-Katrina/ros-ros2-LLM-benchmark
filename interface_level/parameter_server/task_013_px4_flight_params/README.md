# Task 013 — PX4 Flight Controller Parameter Handling
**Category:** Interface Level / Parameter Server 
**Source:** https://github.com/PX4/px4_ros_com 
**ROS Version:** ROS1 (Noetic)

This task focuses on how PX4-related ROS1 nodes load parameters from the ROS parameter server. 
The benchmark removes parameter-loading lines from real initialization logic.

Students must restore the missing ROS parameter calls based on typical PX4/ROS conventions.

## Versions
- **v01 (Easy):** One missing parameter.
- **v02 (Medium):** Multiple missing parameters and one dependent computation.
- **v03 (High):** Entire parameter group removed; core computation block removed.

## Purpose
To validate whether a system can recover:
1. Correct `rospy.get_param` usage 
2. Naming conventions (`~param_name`) 
3. Parameter-dependent processing logic 

## Expected Outcome
- After restoring the missing parameters, the node logs the rate or control output correctly. 
- Node runs without `KeyError` for missing parameters. 
- Feedback values reflect restored parameter logic in all versions.
