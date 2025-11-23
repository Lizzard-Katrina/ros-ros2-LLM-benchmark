# Task 010 — Controller Manager Services

## Description
This task tests whether an LLM can complete a ROS1 client node interacting with controller_manager services:
- List controllers
- Switch controllers (start/stop)
- Verify state

## Why these blanks?
- **Entire try block**: service call sequence must be understood
- **Service proxies and response handling**: AI must fill correctly
- **CMakeLists/package.xml**: verify understanding of catkin build and dependencies

## Expected Outcome
- controller_manager launch successfully
- client lists controllers and switches state
- Scripts build and run without errors
