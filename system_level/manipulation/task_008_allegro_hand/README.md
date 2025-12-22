# Task 008: Allegro Hand V5 ROS1

## Overview
- ROS1 support for Allegro Hand V5 (4-finger robotic hand)
- Fine-manipulation only (not full arm)
- Connect driver → read joint state → send torque commands → observe hand motion

## Files
- `ros1_code/src/allegro_hand_node.py`: Main node to communicate with Allegro Hand driver.
- `ros1_code/src/demo_get_joint_state.cpp`: Demo to subscribe to joint states and check correctness.

## Expected Outcome
- Running `allegro_hand_node.py` should publish `JointState` messages to `allegro_hand/joint_states`.
- `demo_get_joint_state.cpp` should print current joint positions and allow basic integration-level checks.
- Torque commands sent via `write_torque()` should move fingers (simulated or real, depending on hardware).

## Docker
- `docker/Dockerfile` and `docker/setup.sh` used for environment setup.
