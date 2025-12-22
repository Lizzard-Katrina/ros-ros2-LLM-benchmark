# Task 006: Unknown Environment Exploration + Mapping

## Level
Integration Level — Navigation Stack Correctness

## Description
This task evaluates robot navigation in an initially unknown environment.

Flow:
1. Run SLAM to explore environment (TODO: run_slam_todo.py)
2. Save the generated map (TODO: save_map_todo.py)
3. Start localization using AMCL (TODO: start_localization_todo.py)
4. Send navigation goals using move_base (TODO: send_goal_todo.py)

## TODOs

### run_slam_todo.py
**Goal:** Initialize SLAM and explore environment
**Expected Outcome:** Map is being built in real-time

### save_map_todo.py
**Goal:** Save map after exploration
**Expected Outcome:** Map file is saved to disk

### start_localization_todo.py
**Goal:** Start AMCL localization
**Expected Outcome:** Robot localizes correctly on saved map

### send_goal_todo.py
**Goal:** Send navigation goals
**Expected Outcome:** Robot reaches goals using the generated map

## Validation

Single TODO validation:
- Check SLAM node runs and generates map
- Verify map file is saved
- Check AMCL localization updates robot pose
- Mock sending a goal and verify MoveBaseGoal message

Full pipeline validation:
- Run SLAM in Gazebo or real robot
- Save map
- Start AMCL localization
- Send navigation goals
- Verify robot navigates correctly and avoids obstacles
