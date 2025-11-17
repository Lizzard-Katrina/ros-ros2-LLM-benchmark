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
