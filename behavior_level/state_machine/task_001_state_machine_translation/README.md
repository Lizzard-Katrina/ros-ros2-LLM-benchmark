# Task: Turnstile FSM ROS Translation

## Overview
Benchmark task for evaluating LLM ROS1 → ROS2 translation.

### TODOs and expected outcomes
- `state_locked.py` → LLM translates ROS1 coin check & publisher to ROS2. Expected: ROS2 node waits for coin, publishes "Unlocked".
- `state_unlocked.py` → Translate ROS1 push check & publisher to ROS2. Expected: ROS2 node waits for push, publishes "Locked".
- `state_transition.py` → Translate SMACH state additions & transitions to ROS2-compatible. Expected: ROS2 SMACH executes same LOCKED/UNLOCKED flow.

## Build & Run
Build Docker and run ROS1/ROS2 nodes as before.
