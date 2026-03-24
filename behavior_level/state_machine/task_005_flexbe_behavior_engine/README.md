# Benchmark Task 005: Nav2 MoveBase Migration (FlexBE Proxy Pattern)

## 1. Brief Description
This task involves migrating a deprecated ROS 1 navigation state (`MoveBaseState`) to the modern **ROS 2 Nav2 (`MapsToPose`)** architecture within the FlexBE framework. The original source is "broken," containing placeholders and legacy `actionlib` structures. 

The challenge lies in correctly implementing the **FlexBE ProxyActionClient**, which abstracts ROS 2's asynchronous `Future` management into a polling-based state machine logic.
---
source code
```https://github.com/FlexBE/generic_flexbe_states/blob/ros2-devel/flexbe_navigation_states/flexbe_navigation_states/move_base_state.py```

## 2. Hollowing Strategy
We employ a **Strict Double-Hole** strategy with specific architectural constraints:

- **Hole A (`__init__`)**: Removes the Action Client initialization. 
  - *Goal*: Test if the LLM can map the correct Nav2 Action type (`MapsToPose`) and use the absolute topic path (`'/navigate_to_pose'`) as required by the style guide.
- **Hole B (`execute`)**: Removes the polling and outcome logic. 
  - *Goal*: This is the "Killer" hole. It tests if the LLM mistakenly tries to manually manage ROS 2 `Futures` (e.g., `get_result_async`), which leads to crashes in FlexBE. It forces the use of `self._client.has_result()`.

## 3. Oracle Test Design & Expected Outcomes

| Test Case | Design Logic | Expected Outcome (Success Pattern) |
| :--- | :--- | :--- |
| `test_no_ros1_remnants` | Uses regex to ensure legacy terms like `actionlib` or ROS 1 constants are purged, while safely ignoring the class name `MoveBaseState`. | No `actionlib` or `GoalStatus.SUCCEEDED` in active logic. |
| `test_nav2_topic_naming` | Enforces the use of the absolute topic name defined in the TODO. | Exact string `'/navigate_to_pose'` must be present. |
| `test_flexbe_proxy_compliance` | **(Critical)** Specifically forbids manual future management (`get_result_async`) which causes `NoneType` errors in FlexBE. | Use of `has_result()` and NO manual `future` variables. |
| `test_ros2_goal_status_constants` | Validates that the LLM uses the correct ROS 2 `action_msgs` constants. | Use of `STATUS_SUCCEEDED` instead of legacy `SUCCEEDED`. |
| `test_placeholder_removal` | Checks if the LLM successfully removed the `NotImplementedError` and "deprecated" warnings. | Total absence of `NotImplementedError`. |
