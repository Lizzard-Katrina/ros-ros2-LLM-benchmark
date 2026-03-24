# Task 006: Dynamic Parameter Async-Sync Bridge

## 🎯 Brief Description
This task involves implementing the core communication bridge in `rqt_reconfigure` for ROS 2. Unlike ROS 1, ROS 2 parameter operations are asynchronous services. To maintain a responsive GUI while providing a predictable programming interface, the model must implement a "synchronous-wrapper" around asynchronous service calls using Python threading primitives.
---
source code file:
```https://github.com/ros-visualization/rqt_reconfigure/blob/rolling/src/rqt_reconfigure/param_api.py```
## 🕳️ The "Hole" Strategy: Why this was removed?
The method `_call_service` in `param_api.py` was selected for excavation because it represents a critical **concurrency pattern** in ROS 2:
1.  **Deadlock Prevention**: It tests whether the model understands that `client.call()` (synchronous) should be avoided in GUI executors to prevent deadlocks.
2.  **Thread Synchronization**: It forces the model to use a `threading.Event` to bridge the gap between a `rclpy` `Future` and a blocking return.
3.  **Error Propagation**: It requires the model to map asynchronous failures (timeouts, null results) into a specific exception class (`AsyncServiceCallFailed`) with mandatory semantic hints.

## ✅ Oracle Test Design & Expected Outcomes

The Oracle tests use pattern matching to verify the architectural integrity and instruction following of the generated code.

| Test Case | Design Rationale | Expected Outcome |
| :--- | :--- | :--- |
| `test_deadlock_avoidance` | Ensures the model uses the non-blocking `call_async()` and avoids the dangerous `.call()`. | Code must contain `call_async` and MUST NOT contain `.call(`. |
| `test_sync_primitive_choice` | Validates the choice of `threading.Event` as the bridge, which is the standard pattern for this tool. | Presence of `Event()` and `.wait()`. |
| `test_service_readiness` | Checks for ROS 2 service discovery logic to prevent calling non-existent services. | Presence of `wait_for_service` and/or `service_is_ready`. |
| `test_future_callback_logic` | Verifies the model knows how to unblock the waiting thread via a callback. | Presence of `add_done_callback` and `.set()` inside the logic. |
| `test_instruction_hints` | Validates strict adherence to the provided strings in the TODO for UI error reporting. | Exact matches for `"timed out waiting for service"` and `"the target node may not be spinning"`. |
| `test_no_ros1_legacy` | Scans for "lazy" migration artifacts like `rospy` or ROS 1 specific threading styles. | **Total absence** of `rospy`, `dynamic_reconfigure`, or `ServiceProxy`. |
| `test_future_result_access` | Ensures the final service response is actually retrieved from the future object. | Presence of `future.result()`. |

## 🛠️ Usage for Benchmarking
To pass this task, the model must not only generate functional code but also adhere to the **specific technical constraints** that prevent UI freezes. A model that implements a simple `while not future.done(): pass` loop will fail the "Style" checks, as busy-waiting is an anti-pattern in high-quality ROS 2 development.
