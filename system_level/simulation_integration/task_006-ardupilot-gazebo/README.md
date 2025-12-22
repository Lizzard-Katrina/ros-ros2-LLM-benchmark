# ArduPilot SITL + Gazebo + ROS1 (MAVROS) Plugin

## Overview
This plugin integrates ArduPilot SITL with Gazebo simulation and ROS1 using MAVROS.  
It enables closed-loop control from autopilot commands to simulated vehicle actuators and sensor feedback.

## Task Objective
- Build the ArduPilot Gazebo plugin.
- Launch Gazebo with the vehicle model.
- Run MAVROS to bridge ROS topics for control and sensors.
- Implement closed-loop logic: servo commands → motor forces → state feedback.

## Key Files
- `include/ArduPilotPlugin.hh` — Header with TODOs for closed-loop functions.
- `src/ArduPilotPlugin.cc` — Main implementation with parameter loading and function placeholders.
- `metadata.json` — Task metadata.

## Usage
```bash
cd <workspace>
catkin_make
source devel/setup.bash

roslaunch ardupilot_gazebo gazebo_iris.launch
roslaunch mavros px4.launch
## Expected Outcome

- `LoadControlChannels()`, `LoadImuSensors()`, `LoadGpsSensors()`, `LoadRangeSensors()`, `LoadWindSensors()` correctly load respective sensor and actuator configurations from SDF.
- Closed-loop update correctly integrates:
  - Reading servo commands from SITL (`ReceiveServoPacket()`)
  - Applying forces/velocities to Gazebo joints (`UpdateMotorCommands()`)
  - Updating PID controllers (`ApplyMotorForces()` / `ResetPIDs()`)
  - Creating simulation state JSON (`CreateStateJSON()`)
  - Sending state back to SITL (`SendState()`)
- `InitSockets()` initializes communication with SITL.
- After implementation, the simulated vehicle responds correctly to autopilot commands, with sensors and actuators synchronized with Gazebo simulation.
