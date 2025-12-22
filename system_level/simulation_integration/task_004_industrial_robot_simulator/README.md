# Industrial Robot Simulator (ROS1)

## Overview
This task simulates industrial robot controller semantics using ROS1.  
It allows testing integration between an industrial robot controller and a simulated environment, verifying message formats, trajectory commands, and feedback topics.

## Workflow
1. Start the simulator using the provided launch file:
```bash
roslaunch industrial_robot_simulator robot_interface_simulator.launch
```
2. The simulator nodes accept motion commands and publish robot state.

3. The joint_trajectory_action node provides an actionlib interface for high-level robot motion control.

4. Use client nodes to send joint trajectories and monitor robot state.
##Scaffold

Launch file: industrial_robot_simulator/launch/robot_interface_simulator.launch

Nodes:

industrial_robot_client/src/generic_joint_trajectory_action_node.cpp

industrial_robot_client/src/generic_robot_state_node.cpp
