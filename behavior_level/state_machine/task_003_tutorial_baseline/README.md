# Benchmark Task 003: Sim Monitor State (NRP Loop & Time Migration)

## 1. Brief Description
This task involves migrating a specialized **Simulation Monitor State** from the Neurorobotics Platform (NRP) SMACH tutorial. The state is designed to monitor simulation time in a non-blocking loop and trigger a Gazebo service once a specific time threshold is reached.

The primary challenge is moving from the synchronous, global `rospy.Time` and `rospy.Rate` to the ROS 2 `rclpy` lifecycle, which requires explicit **Executor Spinning** (`spin_once`) and manual **Duration Math** to handle nanosecond-to-second primitives.
---
source tutorial:
```https://neurorobotics.net/Documentation/legacy/nrp/modules/ExDBackend/hbp_nrp_backend/tutorials/state_machines.html```


## 2. Hollowing Strategy (Hole-filling)
The hollowing focuses on the **Execution Logic** within a high-frequency loop to evaluate low-level API handling.

- **Scope:** The entire `execute` method body and the specific argument order in `__init__`.
- **Injected TODO:** A high-precision prompt that mandates:
    - A specific **Constructor Argument Order** (`node` before `model_name`).
    - **Manual Time Conversion:** Forcing the use of `.nanoseconds / 1e9` instead of high-level `Duration` objects to test primitive data handling.
    - **Asynchronous Service Pattern:** Requiring `call_async` followed by `spin_until_future_complete`.
    - **Style Constraints:** Explicitly forbidding Python Type Hints and legacy `rospy` patterns.

### Why this approach?
This prevents the LLM from using "shortcuts" (like ROS 2 `Duration` object comparisons) that might be correct in isolation but bypass the benchmark's goal: testing if the AI can follow strict architectural constraints and handle ROS 2 time primitives manually.

## 3. Oracle Test Design & Expected Outcomes

The Oracle tests for Task 003 are **Strict** and **Opinionated** to ensure the generated code perfectly matches the target system's requirements.

| Test Case | Design Logic | Expected Outcome (Success Pattern) |
| :--- | :--- | :--- |
| `test_constructor_argument_order` | Ensures the node handle is the first positional argument after `self`. Prevents runtime initialization crashes. | `def __init__(self, node, model_name...` |
| `test_clock_and_manual_math` | Forces manual nanosecond-to-second conversion. Detects if the LLM correctly identified that ROS 2 `Time` subtraction returns an object requiring unit conversion. | `.nanoseconds / 1e9` |
| `test_service_future_logic` | Verifies the full ROS 2 async service lifecycle: Request → Call Async → Spin until Future Complete → Check Result. | `call_async` AND `spin_until_future_complete` |
| `test_executor_spinning_in_loop` | Checks for `rclpy.spin_once` inside the `while` loop. Without this, the node's clock would stop updating, causing a deadlock in simulation. | `while` loop containing `rclpy.spin_once` |
| `test_preemption_check` | Ensures the state remains interruptible by the SMACH executive during long-running wait loops. | `self.preempt_requested()` |
| `test_logging_migration` | Confirms the removal of `rospy.loginfo` in favor of the node-specific logger. | `self.node.get_logger().info(...)` |
| `test_no_type_hints` | Enforces a specific code style (no `: Node`) to ensure compatibility with legacy-style Python environments. | Regex search for `node: \w+` returns `None`. |
