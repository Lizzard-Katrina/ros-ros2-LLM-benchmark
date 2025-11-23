# Task 008 — Navigation: clear_costmaps service

## Description
This task tests whether an LLM can correctly complete a ROS1 client node calling /move_base/clear_costmaps service.

## Why these blanks?
- **Entire try block**: AI must understand service call sequence
- **ServiceProxy and response**: must be handled correctly
- **CMakeLists/package.xml**: test understanding of catkin build and dependencies

## Expected Outcome
- Map server + move_base launch
- clear_costmaps client successfully clears costmaps
- Scripts build and run without errors
