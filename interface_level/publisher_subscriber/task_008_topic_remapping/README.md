# Task 008 — Topic Remapping in Launch Files

## Goal
Evaluate whether the AI can correctly handle topic remapping in ROS1 launch files and understand the optional ROS2 launch equivalent.

## Description
- A publisher and a subscriber node communicate via a topic.
- The topic is remapped at runtime using `<remap>` in launch XML.
- The subscriber should receive messages on the remapped topic.

## Requirements
- Fill in missing topic names (`_____`) in ROS1 nodes and launch file.
- Optional: fill ROS2 nodes and launch remapping scaffold.

## Directory Structure
- ros1_code/: ROS1 nodes + launch file
- expected_ros2_code/: ROS2 scaffold (optional)
- docker/: ROS1 + ROS2 environment
- tests/: placeholder for future validation
