# Task 007: SMACC Async Behavior Migration (C++)

## 1. Brief Description
This task involves migrating the `SmaccAsyncClientBehavior` class from ROS 1 to ROS 2. This component is responsible for managing asynchronous lifecycle threads (`onEntry` and `onExit`) within the SMACC state machine framework. The challenge lies in transitioning from legacy ROS 1 global macros and blocking spin logic to a thread-safe, executor-aware ROS 2 architecture.
---
source code
```https://github.com/robosoft-ai/SMACC/blob/noetic-devel/smacc/src/smacc/smacc_client_async_behavior.cpp```

## 2. Hollowing Strategy
We use a **Functional-Logic Hollowing** approach, removing the internal thread management in two key areas:
- **Hole A (Async Execution - `executeOnEntry`)**: Removes the `std::async` launch logic. Tests whether the LLM correctly handles the finish event callback and uses ROS 2 logging.
- **Hole B (Thread Joining - `executeOnExit`)**: Removes the polling loop that joins the entry thread. This is a "trap" to see if the LLM reverts to legacy `ros::spinOnce` or forgets to launch the subsequent `onExit` thread.

## 3. Oracle Test Design & Expected Outcomes

| Test Case | Design Logic | Expected Outcome |
| :--- | :--- | :--- |
| `test_api_migration` | Scans for `ROS_` prefixes and `ros::` namespaces. | Complete removal of ROS 1 macros; adoption of `RCLCPP_` and `getLogger()`. |
| `test_executor_safe` | Detects `ros::spinOnce` or `ros::ok`. | Must use `rclcpp::ok()`. Manual spinning is forbidden to prevent SMACC executor deadlocks. |
| `test_functional_flow` | Checks for the presence of `onExit()` and `onExitThread_`. | **Critical**: Fails if the LLM "cleans up" the code by deleting the `onExit` launch logic. |
| `test_future_polling` | Ensures `wait_for` and `future_status` are used. | Implementation must use non-blocking polling to keep the state machine responsive. |
| `test_memory_safety` | Analyzes Lambda captures `[=]` vs `this->` usage. | Warning/Failure if `this` is captured by value without lifetime protection (e.g., `shared_from_this`). |
| `test_proper_rate` | Checks for `rclcpp::Rate` migration. | Replaces `ros::Rate` to ensure timing consistency in ROS 2. |
