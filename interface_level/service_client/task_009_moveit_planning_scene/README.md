# Task 009 — MoveIt: Apply Planning Scene Service

## Description
This task tests whether an LLM can complete a ROS1 client node calling /apply_planning_scene.

## Why these blanks?
- **Entire try block**: service call sequence must be understood
- **PlanningScene object**: AI must fill robot_state, collision_objects, and set is_diff
- **ServiceProxy and response**: correct handling required
- **CMakeLists/package.xml**: test understanding of catkin build and dependencies

## Expected Outcome
- move_group + robot_state_publisher launch
- apply_planning_scene client sends PlanningScene diff successfully
- Scripts build and run without errors
