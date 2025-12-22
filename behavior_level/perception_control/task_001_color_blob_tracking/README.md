# Task 001: Color Blob Tracking + Simple Mobile Control

## Description
Detect a colored blob using a camera and command the robot to move toward it.  
Input: `/camera/image_raw`  
Output: `/cmd_vel`  

## Expected outcome for each TODO
1. **constructor_init.cpp**:
   - Blob tracking parameters initialized (color bounds, max speed)
   - Node compiles and subscribers/publishers are ready
2. **blob_detection.cpp**:
   - Blob detected in image
   - `blob_center_` and `blob_detected_` updated
3. **control_command.cpp**:
   - Blob position converted to `cmd_vel` commands
   - Robot moves toward the blob; stops if not detected

## Validation
- Each TODO can be tested with static or simulated camera input
- Full validation: robot follows a moving colored object in simulation or real camera feed
