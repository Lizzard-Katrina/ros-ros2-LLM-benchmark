# Task 004: Mobile Platform + LiDAR → Obstacle Avoidance

## Description
Detect obstacles from LiDAR and command the mobile robot to avoid collisions.  
Input: `/scan`  
Output: `/cmd_vel`

## Expected outcome for each TODO

1. **constructor_init.cpp**:
   - Node parameters initialized (min_distance, speeds)
   - Subscribers/publishers set up
   - Node compiles successfully

2. **obstacle_detection.cpp**:
   - LiDAR scan processed
   - `obstacle_detected_` true if obstacle within `min_distance_`

3. **velocity_publisher.cpp**:
   - Robot stops or turns if obstacle detected
   - Moves forward otherwise
   - Velocity commands published to `/cmd_vel`

## Validation
- Each TODO can be tested with recorded LiDAR scans
- Full system: robot avoids obstacles in simulation or real environment
