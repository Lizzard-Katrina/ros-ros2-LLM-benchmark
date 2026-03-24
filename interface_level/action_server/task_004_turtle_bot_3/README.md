# Task 004: ROS1 → ROS2 Action Server Migration (Lifecycle Correctness)

## Task Overview

This task evaluates an LLM’s ability to translate a **ROS1 Action Server** into a **semantically correct ROS2 Action Server**, with emphasis on **action lifecycle correctness** rather than superficial API replacement.

Unlike simple publisher/subscriber or service migration tasks, ROS2 actions introduce a fundamentally different execution model based on **goal handles**, **explicit lifecycle transitions**, and **iterative feedback publication**. This task is designed to expose failures where a model produces code that appears syntactically valid but violates core ROS2 action semantics.

---

## Input (ROS1 Reference)

We construct the orignal code block and form the task referencing a turtlebot3 example from 
```https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_examples/```

The original ROS1 implementation uses `actionlib.SimpleActionServer` and follows a canonical ROS1 action pattern:

- Action name: `head_action`
- Execution callback: `execute_cb(goal)`
- Iterative feedback publication using `publish_feedback`
- Blocking execution using `rospy.sleep`
- Terminal success state set via `set_succeeded(result)`

This reference defines a **long-running action** with explicit progress reporting.

---

## Expected Output (ROS2 Target)

The translated ROS2 implementation is expected to:

- Instantiate a `rclpy.action.ActionServer`
- Implement an execution callback that accepts a `goal_handle`
- Follow ROS2-native action lifecycle semantics:
  - Access goal data via `goal_handle.request`
  - Publish feedback iteratively during execution
  - Populate the result object **before** setting the terminal goal state
  - Explicitly transition the goal using `goal_handle.succeed()`, `abort()`, or `canceled()`
- Avoid service-style return semantics (i.e., no `return result`)

---

## Evaluation Methodology

This task is evaluated using **static oracle tests** that analyze the translated ROS2 source code. The tests focus on **semantic properties** of action execution rather than runtime behavior.

Before analysis, the oracle:
- Removes Markdown code fences
- Strips all comments
- Operates purely on executable source text

---

## Oracle Test Groups

### Test Group 0: ROS1 Artifact Removal

Ensures that no ROS1 concepts remain in the translated code.

The following artifacts are forbidden:
- `rospy`
- `actionlib`
- `SimpleActionServer`
- `rospy.spin`
- `rospy.sleep`
- `rospy.loginfo`

---

### Test Group 1: ROS2 ActionServer Construction

Verifies that:
- A `rclpy.action.ActionServer` is instantiated
- An execution callback is defined with a `goal_handle` parameter

This enforces correct ROS2 action server structure.

---

### Test Group 2: Execute Callback Semantics (Critical)

Ensures that action execution does **not** behave like a ROS service.

The oracle flags service-style returns such as:
- `return True`
- `return False`
- `return goal`
- `return result`

ROS2 action execution must be lifecycle-driven, not return-driven.

---

### Test Group 3: Goal Handling Semantics

Checks that goal data is accessed via:

```python
goal_handle.request.<field>

## To run this test

1. docker build -t ros2-test .

2. docker run -it --rm ros2-test
3. python3 -m pytest src/task_004/test/test_oracle_ros2.py
