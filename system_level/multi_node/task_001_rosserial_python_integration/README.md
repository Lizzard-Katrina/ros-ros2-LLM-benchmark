# Benchmark Task: ROS 1 to ROS 2 Migration (rosserial_python)

## 1. Brief Description
This benchmark evaluates an AI's capability to migrate a legacy ROS 1 package (`rosserial_python`) to ROS 2 (Humble/Foxy). The core challenge is not merely a syntax translation, but a **structural refactoring** from a globally-coupled state (`rospy`) to a **Dependency Injection (DI)** pattern. The migrated Logic Engine must operate seamlessly using an externally provided `rclpy.Node` instance without managing its own lifecycle or global state.

---
source code file:
```https://github.com/ros-drivers/rosserial/blob/noetic-devel/rosserial_python/src/rosserial_python```

## 2. Design of Code Blanks (Gap-filling Strategy)

The benchmark is split across two files to test **Architectural Decoupling** and **Inter-file Consistency**:

### File A: `serial_node.py` (Node Wrapper Layer)
* **Selection Logic**: This file serves as the system "Shell." It handles the ROS 2 boilerplate.
* **Gap Design**: 
    * **Parameter Handling**: Tests the model's knowledge of the mandatory `declare_parameter` before `get_parameter` flow.
    * **Instantiation**: Tests if the model correctly injects the node instance (`self`) into the Logic Engine.
* **Coupling Goal**: To ensure the model understands that the Node is the provider of resources, not just a container.

### File B: `SerialClient.py` (Logic Engine Layer)
* **Selection Logic**: This is the system "Brain" containing complex serial protocols and message serialization.
* **Gap Design**: 
    * **Constructor Interface**: Forces the model to modify the `__init__` signature to accept a `Node` object.
    * **Clock & Logging**: Replaces global `rospy.Time` and `rospy.loginfo` with node-specific calls (`node.get_clock().now()`).
* **Coupling Goal**: To evaluate if the model can purge global dependencies and rely entirely on the injected interface.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle tests utilize semantic regex matching to enforce strict ROS 2 best practices.

| Test Case | Design Philosophy | Expected Outcome |
| :--- | :--- | :--- |
| `test_anti_leakage_rospy` | **Cleansing Check**: Ensures no legacy code or imports remain. | Zero occurrences of the string `rospy` in code or comments. |
| `test_parameter_pattern` | **API Specification**: ROS 2 parameters are more rigid than ROS 1. | Usage of `declare_parameter()` followed by `.get_parameter().value`. |
| `test_dependency_injection` | **Linkage Integrity**: Verifies the "Handshake" between files. | `SerialClient` must be instantiated with `self` or `node=self`. |
| `test_node_clock_usage` | **Timing Accuracy**: Prevents "lazy" migration using system `time.time`. | Direct chained calls to `node.get_clock().now()` for all timestamps. |
| `test_executor_spin` | **Lifecycle Management**: Tests the transition from ROS 1 global spin. | Use of `rclpy.spin(node)` in the main entry point. |
| `test_absence_of_global_init` | **Architectural Purity**: Ensures the Engine remains a library, not a node. | No calls to `rclpy.create_node()` within the `SerialClient.py` file. |

### Expected Outcome (10/10 Score)
A successful migration demonstrates:
1.  **Zero Global State**: No internal node creation; all ROS 2 features are accessed via the injected node.
2.  **Chained API Calls**: Proper usage of the ROS 2 `Node` API for parameters, logging, and clocks.
3.  **Clean Execution**: A fully functional wrapper that drives the engine using standard ROS 2 executors.

