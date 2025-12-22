# Task 002: UR5 Pick-and-Place in Gazebo

## Level
Integration-Level Correctness — Manipulation

## Source
https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation

## Task Description
This task evaluates whether a robotic manipulation pipeline is correctly integrated:
from perception to motion planning to execution.

## Pipeline
Spawn bricks → Detect LEGO → Plan motion → Pick → Place → Repeat

---

## TODO Files and Source Mapping

### 1. spawn_bricks_todo.launch
- Source: ur5_gazebo/launch/spawn_bricks.launch
- Goal: Spawn LEGO bricks correctly in Gazebo
- Expected Outcome: Bricks appear at correct positions

### 2. lego_detector_todo.py
- Source: ur5_vision/scripts/lego_detector.py
- Goal: Detect LEGO bricks and publish pose
- Expected Outcome: /lego_pose topic publishes valid poses

### 3. motion_planner_todo.py
- Source: ur5_moveit/scripts/motion_planner.py
- Goal: Plan motion to detected brick
- Expected Outcome: Valid MoveIt trajectory is executed

### 4. pick_and_place_todo.py
- Source: ur5_control/scripts/pick_and_place.py
- Goal: Execute grasp and placement sequence
- Expected Outcome: Brick is picked and placed, robot returns home

---

## Validation

### Per-TODO Validation
- Each node runs independently without runtime errors
- Topics are correctly published/subscribed

### Full Integration Validation
- Run Gazebo simulation
- Robot repeatedly picks and places LEGO bricks
- No deadlocks between perception, planning, and execution
