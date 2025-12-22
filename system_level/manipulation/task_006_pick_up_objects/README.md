# 006 - TurtleBot3 Pick & Place Objects (Benchmark)

## Overview
This benchmark evaluates LLMs on reasoning about ROS1 behavior tree code for a TurtleBot3 robot that picks and places objects in Gazebo.

**Pipeline:**
Detect object → Behavior Tree triggers pick → Pick object → Place into box → Loop

**Source code:** All under `ros1_code/src/`.

### Subtasks

| Subtask | Source File | Description |
|---------|-------------|------------|
| 006-1   | pickup_behaviors_node.py | Check object and write to blackboard |
| 006-2   | pickup_behaviors_node.py | Get object behavior |
| 006-3   | pickup_behaviors_node.py | Let object behavior |
| 006-4   | turtlebot_controller_node.py | Move robot to target point |
| 006-5   | manage_objects_node.py | Manage objects in Gazebo |

