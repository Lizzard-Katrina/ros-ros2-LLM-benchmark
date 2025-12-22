# Task 005: RGB-D / Stereo Camera → Point Cloud → Mobile Control

## Description
Use RGB-D or stereo camera to generate point cloud and command robot motion for obstacle avoidance.  
Flow: `/camera/depth/points` → point cloud processing → `/cmd_vel`

## Expected outcome for each TODO

1. **constructor_init.cpp**
   - Node parameters initialized
   - Subscriber / Publisher set up
   - Node compiles successfully

2. **pointcloud_processing.cpp**
   - Point cloud processed
   - `obstacle_detected_` set true if points within threshold

3. **motion_publisher.cpp**
   - Robot moves or avoids obstacles
   - Commands published to `/cmd_vel`

## Validation
- Each TODO can be tested independently
- Full system: robot navigates avoiding obstacles in simulation or real-world
