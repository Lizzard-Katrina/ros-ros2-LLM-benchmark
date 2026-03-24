# Benchmark Task 004: Task Manager Wrapper (Architecture & Concurrency)

## 1. Brief Description
This task involves the migration of `TaskSmach.py`, a core orchestration layer that wraps SMACH states into a Task Manager framework. Unlike simple state migrations, this task requires bridging SMACH’s **blocking execution** with ROS 2’s **asynchronous callback model**.

The primary challenge is to prevent node deadlocks while maintaining the ability to monitor task status and handle system-level interrupts (SIGINT) gracefully.
---
source code:
```https://github.com/cedricpradalier/ros_task_manager/blob/master/src/task_manager_lib/src/task_manager_lib/TaskSmach.py#L20```


## 2. Hollowing Strategy (Hole-filling)
To evaluate the LLM's system architecture capabilities, we employ a **Double-Hole** strategy:

- **Hole A (`TaskState.execute`):** Removes the logic for triggering tasks and waiting for their results. This tests if the LLM can handle asynchronous status polling and map complex exceptions to state machine outcomes.
- **Hole B (`MissionStateMachine.run`):** Removes the entire execution lifecycle. This is the "Killer" hole—it forces the LLM to implement a concurrency model (Threads + Executors) that allows the State Machine to run without starving the node's internal communication.

### Why this approach?
A naive migration (directly copying ROS 1 thread logic) will cause a **permanent deadlock** in ROS 2 because the service/topic callbacks needed to update task status will never be processed by the executor while the State Machine is blocking the thread.

## 3. Oracle Test Design & Expected Outcomes

| Test Case | Design Logic | Expected Outcome (Success Pattern) |
| :--- | :--- | :--- |
| `test_concurrency_architecture` | Checks for the presence of an Executor or Threading mechanism. Ensures the node isn't "frozen" during execution. | Inclusion of `MultiThreadedExecutor` or `threading.Thread`. |
| `test_outcome_logic_mapping` | Verifies that the task's lifecycle (Timeout, Failure, etc.) is correctly translated into the SMACH outcome set. | Presence of `TASK_TIMEOUT`, `TASK_FAILED`, and `TASK_INTERRUPTED`. |
| `test_preemption_handling` | Ensures that system signals (SIGINT) are caught to trigger `request_preempt()`, preventing "zombie tasks" in simulation. | Explicit call to `sm.request_preempt()` or `self.tc.stopTask()`. |
| `test_introspection_node_handle` | Verifies the use of the ROS 2 version of the Introspection Server API, which requires a node handle. | `IntrospectionServer(..., self.node, ...)` |
| `test_strict_style_no_hints` | Enforces a specific "clean" Python style without type hints to maintain consistency with the legacy codebase. | No usage of `: Node`, `: int`, or `-> None`. |
| `test_ros2_handle_usage` | Validates the complete removal of `rospy` and the correct use of the Node's logger. | `self.node.get_logger().info()` and NO `rospy` strings. |
| `test_rclpy_init_protection` | **(Critical)** Checks for "Double-Initialization" safety. Blindly calling `rclpy.init()` in a library component is a major error. | Code must use `if not rclpy.ok(): rclpy.init()`. |
