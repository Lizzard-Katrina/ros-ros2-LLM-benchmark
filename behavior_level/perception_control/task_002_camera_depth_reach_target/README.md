# Task 002: Camera + Depth → Reach Target Point

## Description
Detect an object using a depth camera and move a robotic arm to reach/pick it.  
Input: `/camera/depth/image_raw`  
Output: MoveIt arm commands

## Expected outcome for each TODO

1. **constructor_init.cpp**:
   - MoveIt move_group initialized
   - Planning parameters (time, velocity, acceleration) set
   - Node compiles and subscribers ready

2. **object_detection.cpp**:
   - Object detected in depth camera image
   - `target_pose_` updated
   - `target_detected_` set true if object found

3. **motion_planner.cpp**:
   - Trajectory generated and executed to reach target_pose_
   - Robot arm moves toward target; stops if target not detected

## Validation
- Each TODO can be tested separately with simulated depth data or recorded bag
- Full validation: robot arm reaches a specified target using MoveIt in simulation or real robot
