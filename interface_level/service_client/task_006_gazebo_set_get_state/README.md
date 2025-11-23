# Task 006 — Gazebo Set/Get Model State

## Description
This task tests whether an LLM can correctly complete set/get model state ROS1 service nodes for Gazebo simulation.

## Why these blanks?
- **Entire try blocks**: require understanding ROS service call structure
- **Service names & message types**: prone to error
- **ModelState object**: AI needs to fill model name, pose, twist correctly
- **CMakeLists / package.xml**: test understanding of catkin build

## Expected Outcome
- Gazebo empty world launches
- set_model_state node modifies model state successfully
- get_model_state node retrieves model state successfully
- No build/runtime errors
