# Task 002: TurtleBot3 Patrol Action Server Benchmark

## Overview

This benchmark evaluates the translated ROS2 implementation of the **TurtleBot3 patrol action server**. The original ROS1 action server (`turtlebot3_patrol_server.py`) had the `execute_callback` function containing the core patrol logic, which we excised to allow LLM-based translation.

---
## Sourced project:

- github `https://github.com/ROBOTIS-GIT/turtlebot3/blob/main/turtlebot3_example/turtlebot3_example/turtlebot3_patrol/turtlebot3_patrol_server.py`


---

## Reason for Excising `execute_callback`

The `execute_callback` function performs the following:

1. Determines which patrol pattern to execute (square or triangle) based on the goal.
2. Performs the patrol according to the goal specifications.
3. Publishes feedback messages reflecting patrol progress.
4. Marks the goal as succeeded and returns a result summarizing completion.

We excised this function to allow LLMs to **reconstruct the patrol sequence**, while keeping the rest of the server infrastructure (Node, publishers, subscriptions) intact. This allows benchmarking the correctness of the translated logic.

---

## Oracle Tests

### Test Group 0: ROS1 Artifacts
- **Purpose:** Ensure ROS1 APIs (`rospy`, `actionlib`, `SimpleActionServer`) are removed.
- **Expected outcome:** Translated code contains none of these artifacts.

### Test Group 1: ROS2 Action Server Usage
- **Purpose:** Verify the code uses ROS2 ActionServer (`rclcpp_action` or `rclpy.action`) and retains the `Patrol` action type.
- **Expected outcome:** ROS2 action server library imported and `Patrol` type present.

### Test Group 2: Node and Server Initialization
- **Purpose:** Check that a ROS2 Node is created and the action server is instantiated.
- **Expected outcome:** Node creation detected; ActionServer object bound to callback exists.

### Test Group 3: Execute Callback Semantics
- **Purpose:** Ensure `execute_callback` exists and references the patrol goal.
- **Expected outcome:**  
  - `execute_callback` function defined  
  - References `goal_msg.goal.x` for pattern selection  
  - Uses `goal_handle`  
  - Calls `square()` or `triangle()` to execute patrol sequence  

