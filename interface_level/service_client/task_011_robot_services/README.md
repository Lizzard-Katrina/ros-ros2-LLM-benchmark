# Task 011 — Robot-specific Service Suites

## Description
Complete a partially empty client node that calls robot-specific services in simulation:
- Gripper open/close
- Head movement
- Arm movement (if applicable)

## Why these blanks?
- **Entire try block**: multiple sequential service calls must be correctly handled
- **ServiceProxy and request construction**: AI must understand message types and service parameters
- **CMakeLists/package.xml**: verify understanding of catkin build and robot-specific dependencies

## Expected Outcome
- Simulation launches (Fetch or PR2)
- Client calls multiple services sequentially
- Robot motion/state changes observable in Gazebo or RViz
