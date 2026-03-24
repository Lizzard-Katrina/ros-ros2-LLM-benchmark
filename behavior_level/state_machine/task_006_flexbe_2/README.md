# Task 006: FlexBE Joint State Alignment (Incremental Mapping)

## 1. Brief Description
This task involves migrating a joint-value retrieval state to ROS 2. The core objective is to collect current positions for a specific set of joints from the `/joint_states` topic. 

Unlike simple subscribers, this state must handle **asynchronous, partial, and non-deterministic** data. It must reliably "fill in the blanks" across multiple incoming messages from a buffer until all requested joint values are acquired. The implementation must remain strictly compliant with ROS 2 timing (simulation-ready) and the FlexBE Proxy architecture.
---
source code
```https://github.com/FlexBE/generic_flexbe_states/blob/ros2-devel/flexbe_manipulation_states/flexbe_manipulation_states/get_joint_values_state.py```

## 2. Hollowing Strategy
The task utilizes a **Two-Hole Lifecycle** approach to test data structure initialization and high-frequency message processing:

- **Hole A (Execution Loop - `execute`)**: 
    - **Strategy**: Removes the core message-processing logic within the polling loop.
    - **Goal**: Tests if the LLM can implement a `while` loop to drain the proxy buffer and perform **incremental updates**. It must use dynamic name-to-index mapping instead of assuming a fixed array order.
- **Hole B (State Setup - `on_enter`)**:
    - **Strategy**: Removes tracking initialization and buffer enablement.
    - **Goal**: Tests the understanding of the state lifecycle (starting the buffer) and the use of the correct ROS 2 node clock for synchronization (handling `use_sim_time`).

## 3. Oracle Test Design & Expected Outcomes

| Test Case | Design Logic | Expected Outcome (Success Criteria) |
| :--- | :--- | :--- |
| `test_strict_ros2_clock` | Forbids manual `Clock()` instantiation via Regex. | Must use `self.get_clock().now()`. Failure indicates non-compliance with simulation time. |
| `test_true_buffer_drain` | Ensures the logic drains the **entire** buffer in a single tick. | Presence of a `while` loop combined with `has_buffered`. An `if` statement is insufficient. |
| `test_incremental_update` | **(Critical Logic Check)** Scans for `is None` or `== None` guards before updating. | Prevents overwriting already-found joint data with stale or partial info from subsequent messages. |
| `test_dynamic_mapping` | Detects name-based lookup (e.g., `index`, `zip`, or `dict`). | Strictly forbids hardcoded indices (e.g., `position[0]`). |
| `test_buffer_enable` | Verifies that the proxy buffer is actually turned on. | Presence of `self._sub.enable_buffer` in the `on_enter` method. |
| `test_userdata_flow` | Validates data passing to the FlexBE executive. | Final list must be assigned to `userdata.joint_values`. |
