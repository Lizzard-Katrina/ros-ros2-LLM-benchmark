# CARLA ↔ ROS Bridge (ROS1)

Integrates the high-fidelity CARLA simulator with ROS1 topics for vehicle control and sensor data.

## Description

- Connects CARLA simulator and ROS nodes.
- Supports synchronous and asynchronous modes.
- Publishes sensor topics and subscribes to control commands.
- Manages ego vehicles, other actors, and world information.

## Expected Outcome

- Control commands from ROS (`CarlaControl`) are received and applied in CARLA.
- Actor states are updated every tick and synchronized with ROS topics.
- Clock and status messages are published to ROS.
- Ego vehicle control commands are correctly handled in synchronous mode.
- Bridge supports spawning/destroying objects and retrieving blueprints.

## Usage

```bash
rosrun carla_ros_bridge bridge.py
