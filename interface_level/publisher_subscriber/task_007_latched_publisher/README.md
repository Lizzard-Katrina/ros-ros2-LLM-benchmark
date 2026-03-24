# Task 007 — Latched Publisher

## Goal
Evaluate whether the AI can correctly implement a ROS1 latched publisher and understand the ROS1 → ROS2 mapping (durability=TRANSIENT_LOCAL).

## Description
A publisher node publishes a message once and exits. A subscriber that starts later should still receive the message.

You must:
- Fill the missing `latch` parameter in `ros1_code/latched_publisher.py`.
- Optionally fill the `durability` parameter in `expected_ros2_code/latched_publisher.py`.

## Directory Structure
- ros1_code/: ROS1 publisher node (incomplete)
- expected_ros2_code/: ROS2 scaffold (optional)
- docker/: build ROS1 + ROS2 environments
- tests/: placeholder for future validation


### Oracle test
This oracle test verifies the ROS2 node functionality in three aspects:

1. **Single Message Verification**
   - Publishes a single message
   - Confirms the subscriber receives it
   - Ensures basic publish/subscribe functionality is preserved

2. **Multiple Messages Verification**
   - Publishes multiple messages in sequence
   - Confirms the subscriber receives all of them
   - Validates correct behavior under repeated publishing

3. **Latched / Transient Local Verification**
   - Creates a new subscriber after messages have been published
   - Confirms the new subscriber immediately receives the last published message
   - Ensures ROS1 latched behavior is preserved in ROS2 via `TRANSIENT_LOCAL` QoS
