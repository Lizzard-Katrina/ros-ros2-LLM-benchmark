# Task 007: TM Robot Arms ROS1 Driver & Demo

## Overview
This task benchmarks integration-level correctness of ROS1 driver for TM industrial robot arms.
It covers: connecting the driver, sending scripts/trajectories, and receiving robot status via topics.


## Tasks / Expected Outcome

1. `tm_ros_node.h`
    - `<FILL>` parts test the following:
        - `send_script`: sends a script to robot, response reflects success/failure.
        - `set_positions`: sends joint positions, response verifies execution.
        - `ask_sta`: queries robot status, response matches expected state.
    - Expected outcome: driver functions correctly invoked, proper response set.

2. `demo_get_sct_response.cpp`
    - `<FILL>` part prints or logs sct_response messages.
    - Expected outcome: callback receives correct topic messages published by driver.

## Notes
- Only core driver interface and demo callback are tested; no package or launch files are needed.
- This task is used for LLM benchmark training on ROS1 integration logic.
