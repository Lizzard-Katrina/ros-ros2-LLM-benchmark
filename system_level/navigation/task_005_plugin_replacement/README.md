# Task 005: Plugin Replacement (Planner/Controller)

## Level
Integration Level — Navigation Stack Correctness

## Description
This task evaluates the behavior of the ROS Navigation stack after replacing a planner plugin.

Steps:
1. Replace local/global planner plugin in move_base configuration (TODO)
2. Update launch file to include the new planner YAML (TODO)
3. Send a navigation goal and validate behavior (TODO)

## TODOs

### replace_planner_todo.py
**Goal:** Replace the planner plugin (e.g., DWAPlannerROS)
**Expected Outcome:** move_base uses the new planner; robot path changes accordingly

### update_launch_todo.py
**Goal:** Update launch to load new move_base parameters
**Expected Outcome:** Launch file correctly loads updated YAML; planner replacement active

### send_goal_todo.py
**Goal:** Send a navigation goal
**Expected Outcome:** Robot reaches the goal using the new planner; obstacle avoidance is handled

## Validation

Single TODO validation:
- Check updated YAML contains correct plugin name
- Check launch file includes correct param
- Mock sending a goal and verify MoveBaseGoal pose

Full pipeline validation:
- Launch move_base with updated planner and launch
- Send navigation goal
- Observe robot behavior matches new planner’s expected trajectory
