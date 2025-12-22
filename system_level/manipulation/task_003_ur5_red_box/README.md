# Task 003: Arm IK Integration

## Overview
This task tests the integration correctness of the robotic arm IK solver and trajectory execution.

## Files and TODOs

### ros1_code/robotic_arm_ikfast_moveit_plugin.cpp
- TODO: Fill in the IK solution selection and trajectory generation logic
- Expected outcome: Given a target end-effector pose, generates a valid joint trajectory

### ros1_code/robotic_arm_ikfast_solver.cpp
- TODO: Fill in the logic to select a feasible IK solution from candidate solutions
- Expected outcome: Returns a valid joint configuration for the given pose

### ros1_code/arm_controller_node.cpp
- TODO: Integrate the plugin and solver, publish trajectory
- Expected outcome: The joint trajectory is correctly published and can be visualized in RViz

## Build and Run
```bash
cd docker
docker build -t task_003_arm .
docker run -it task_003_arm
