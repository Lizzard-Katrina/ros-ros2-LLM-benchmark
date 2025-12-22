# Task 006: 3D Sensor + MoveIt → Robotic Arm Perception(Control Chain)

## Description
Use a 3D sensor to perceive objects and control a robotic arm using MoveIt.  
Flow: `/camera/depth/points` → point cloud processing → MoveIt planning → arm command

## Expected outcome for each TODO

1. **constructor_init.cpp**
   - MoveIt interface initialized
   - Subscriber set
   - Node compiles

2. **pointcloud_processing.cpp**
   - Extract target pose from point cloud
   - `new_target_available_` set true

3. **motion_planning.cpp**
   - Compute MoveIt trajectory to target_pose_

4. **arm_command_publisher.cpp**
   - Execute planned motion
   - Arm moves to target
   - `new_target_available_` handled correctly

## Validation
- Each TODO can be tested independently
- Full system: arm moves toward detected object in simulation or real robot
