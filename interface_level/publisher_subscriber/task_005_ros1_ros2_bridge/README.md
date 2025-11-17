# Task 005 — ROS1 to ROS2 Bridge Communication

## Goal
This task evaluates the model's ability to translate ROS1 publisher logic and ROS2 subscriber logic in a cross-system communication scenario using `ros1_bridge`.

## Description
A ROS1 publisher publishes to `/chatter`.  
The ROS1–ROS2 bridge relays the message to ROS2.  
A ROS2 listener node should receive the message.

You must:
- Fill the missing parts in `ros1_code/talker.py`.
- Optionally provide a ROS2 listener implementation.

## Directory Structure
(standard as previous tasks)

## Docker
The Dockerfile builds:
- ROS1 workspace
- ROS2 workspace
- ros1_bridge workspace

Use `docker/setup.sh` to compile all.


