# Task 004: System-Level IPC and Async Patterns

## 1. Brief Description
This benchmark evaluates an LLM's ability to migrate a distributed system of three interacting nodes from ROS 1 (Noetic) to ROS 2 (Humble). The task moves beyond simple API translation by requiring the implementation of a functional "request-process-relay" pipeline. It tests whether the model understands service discovery, asynchronous request handling, and event-driven execution.

---
source code file:
```https://github.com/ros/ros_tutorials/blob/noetic-devel/roscpp_tutorials```

## 2. Design Philosophy of Gaps (TODOs)

### A. The "Service Provider" Gap (`add_two_ints_server.cpp`)
* **Design**: Hollowing out the callback and server initialization.
* **Logic**: Tests the transition to the ROS 2 `Node` class and the requirement for `std::shared_ptr` in service signatures. It checks if the model correctly maps the ROS 1 `.request` and `.response` to the new pointer-based API.

### B. The "Async Controller" Gap (`add_two_ints_client.cpp`)
* **Design**: Removing the logic from node initialization to result retrieval.
* **Logic**: This is the **most critical system-level gap**. In ROS 1, `client.call()` is blocking. In ROS 2, blocking the main thread can deadlock the executor. The model must implement `wait_for_service` (system sync) and `async_send_request` (non-blocking IPC).

### C. The "Pipeline Relay" Gap (`babbler.cpp`)
* **Design**: Removing the entire execution loop.
* **Logic**: Tests the "Paradigm Shift." The model must replace the manual `while(ros::ok())` loop with a `create_wall_timer`. This determines if the model understands that ROS 2 nodes should be managed by an executor (`spin`).

---

## 3. Testcase Design & Expected Results

The Oracle uses pattern matching to verify architectural intent rather than line-by-line syntax.

| Test Case | Design Intent | Expected Result (Pattern) |
| :--- | :--- | :--- |
| **`test_server_callback_signature`** | Ensure compliance with ROS 2 middleware signatures. | Matches `std::shared_ptr` used with `Request` and `Response`. |
| **`test_server_creation_logic`** | Verify the use of the new Node-based service factory. | Presence of `create_service<...>(...)`. |
| **`test_client_wait_for_service`** | **System Sync**: Verify the client checks if the server is alive before requesting. | Presence of `wait_for_service(...)` call. |
| **`test_client_async_request`** | **Async IPC**: Ensure the main thread is not blocked by a service call. | Presence of `async_send_request(...)`. |
| **`test_client_result_handling`** | Verify the model understands how to retrieve data from a `future`. | Matches `.get()`, `spin_until_future_complete`, or `future::wait`. |
| **`test_babbler_timer_paradigm`** | Verify the transition from Procedural to Event-driven code. | Presence of `create_wall_timer` and absence of legacy `Rate/sleep`. |
| **`test_babbler_executor_spin`** | Ensure the node is actually running under an executor. | Presence of `rclcpp::spin(node)` or similar executor call. |
| **`test_absence_of_legacy`** | **Sanity Check**: Ensure zero leakage of ROS 1 headers/macros. | **Negative match**: Fails if `ros::NodeHandle`, `ros::init`, or `ROS_INFO` are found. |

