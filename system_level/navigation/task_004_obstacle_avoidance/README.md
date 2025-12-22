# Task 004: Navigation with Obstacle Avoidance

## Level
Integration Level — Navigation Stack Correctness

## Description
This task evaluates TurtleBot3 navigation in simulation. The navigation stack (`move_base`) plans paths and avoids obstacles based on costmaps.

Core logic for constructing navigation goals is left as a TODO using real TurtleBot3 navigation example code.

## TODO

### TODO_fill_navigation_goal.py
**Goal:** Convert (x, y, yaw) into a `MoveBaseGoal`.
**Expected Outcome:**
- `frame_id` set to `"map"`
- Position and orientation correct
- Navigation stack avoids obstacles when executing the goal

## Validation

Single TODO Validation:
- Call `fill_navigation_goal` with mock inputs
- Validate pose and quaternion

Full Pipeline Validation:
- Launch TurtleBot3 in Gazebo using navigation launch
- Run turtlebot3_obstacle_navigation.py
- Verify robot navigates to goals and avoids obstacles
