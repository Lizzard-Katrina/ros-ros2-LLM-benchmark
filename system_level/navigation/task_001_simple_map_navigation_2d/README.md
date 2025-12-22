# Task 001: Simple Map Navigation (2D Goal)

## Level
Integration Level – Navigation Stack Correctness

## Description
This task evaluates whether an LLM can correctly integrate the ROS navigation stack
to achieve 2D goal-based navigation in a known map.

Flow:
Map → AMCL → move_base → send goal → robot navigates while avoiding obstacles

## TODO Files

### 1. nav_client_init_todo.py
**Goal:** Initialize the move_base action client.

**Expected Outcome:**
- Action client connects to `/move_base`
- No goal is sent before server is ready

---

### 2. nav_goal_construction_todo.py
**Goal:** Construct a valid 2D navigation goal.

**Expected Outcome:**
- Goal is in `map` frame
- Quaternion orientation is valid
- Goal is visible in RViz

---

### 3. nav_goal_execution_todo.py
**Goal:** Send navigation goal and handle result.

**Expected Outcome:**
- Robot starts moving
- `/cmd_vel` is published
- Goal success or failure is logged

## Validation

### Single TODO Validation
- Launch navigation stack
- Run each node independently
- Check topic/action behavior

### Full Pipeline Validation
- Load map and AMCL
- Send 2D goal
- Robot reaches target without collision

## Notes
- ROS navigation modules are NOT modified.
- Only integration logic is evaluated.
