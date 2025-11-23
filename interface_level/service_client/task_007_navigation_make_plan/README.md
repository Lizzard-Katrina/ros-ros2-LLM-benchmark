# Task 007 — Navigation: move_base make_plan

## Description
This task tests whether an LLM can correctly complete a ROS1 client node calling /move_base/make_plan.

## Why these blanks?
- **Entire try block**: service call sequence must be understood
- **PoseStamped start/goal**: AI must set frame_id, positions, orientations
- **Service proxy and response**: correct handling required
- **CMakeLists/package.xml**: test understanding of catkin build and dependencies

## Expected Outcome
- Map server launches
- move_base launches
- make_plan client requests path and receives a valid nav_msgs/Path
- No build/runtime errors
