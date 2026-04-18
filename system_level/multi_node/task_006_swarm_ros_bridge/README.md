k# ROS 2 Migration Benchmark: Task 006 Swarm ROS Bridge (C++)

## 1. Brief Description
This benchmark evaluates the migration of a high-performance C++ bridge node designed for swarm robotics. The node facilitates reliable ROS message transfer over unstable networks using ZeroMQ. The migration is a "System-Level" challenge requiring the transformation of a procedural ROS 1 script into a modern ROS 2 Object-Oriented `rclcpp::Node`. It specifically tests the model's ability to handle C++ templates, message serialization, and smart-pointer-based callbacks without "hallucinating" or "code-spamming".

---
source:
```https://github.com/shupx/swarm_ros_bridge/blob/master/src```


## 2. Hollowing Design & System Coupling

### File A: `bridge_node.hpp` (Interface Definition)
* **Hole:** The entire class member declaration block, including ROS 2 publishers, subscribers, and thread management.
* **Design Intent:** To force the model to design a coherent ROS 2 class structure. It must choose between `GenericPublisher` (for flexibility) or specific templates.
* **System Coupling:** Tightly coupled with the `.cpp` file. If the header uses `std::shared_ptr<T>` while the source implementation expects a raw pointer or a different smart pointer idiom, the build/test will fail.

### File B: `bridge_node.cpp` (Logic Implementation)
* **Hole:** The `sub_cb` template function body and the serialization-to-ZMQ logic.
* **Design Intent:** To test the migration of the core data-path. ROS 1 `ros::serialization` must be replaced with `rclcpp::Serialization<T>` or `SerializedMessage`. 
* **System Coupling:** The callback signature must match the subscription type declared in the header. This tests if the LLM can maintain architectural integrity across two files.

---

## 3. Oracle Test Cases & Expected Outcomes

| Test Case | Design Intent | Expected Outcome |
| :--- | :--- | :--- |
| `test_hpp_inheritance` | Verifies the shift to OO-style ROS 2. | Matches `class BridgeNode : public rclcpp::Node`. |
| `test_cpp_node_usage` | Checks for correct node-handle referencing. | Uses `this->create_publisher` or `node->create_publisher` instead of legacy handles. |
| `test_cpp_serialization` | Detects migration of the serialization layer. | Presence of `rclcpp::Serialization` and absence of `ros::serialization`. |
| `test_cpp_qos_usage` | Evaluates system-level robustness. | Explicit use of `rclcpp::QoS` or `SystemDefaultsQoS` for swarm stability. |
| `test_system_callback_sync` | Validates inter-file type consistency. | Callback signature must use `const T::SharedPtr msg` (ROS 2 standard). |
| `test_anti_leakage` | Detects "lazy" migration or leftover code. | Zero occurrences of `#include <ros/ros.h>` or `ros::` namespace. |
| `test_no_repetition` (Optional) | **Anti-Spamming check**. | Fails if the model generates excessive repetitive lines (e.g., repeating the same include 50 times). |
