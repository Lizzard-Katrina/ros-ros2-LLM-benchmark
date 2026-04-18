# ROS to ROS 2 Migration Benchmark: Recorder & Talker

## 1. Brief Description
This benchmark evaluates an LLM's capability to perform a **full-system migration** from ROS 1 (Noetic) to ROS 2 (Humble/Iron). Unlike simple code-completion tasks, this challenge requires the model to refactor legacy procedural code into modern object-oriented ROS 2 patterns. It focuses on two critical nodes:

* **`talker.py`**: A Python publisher node testing the transition from global `rospy` calls to the `rclpy.node.Node` class, including parameter handling and non-blocking timers.
* **`recorder.cpp`**: A complex C++ recorder engine testing high-level systems programming, specifically **Type-Agnostic (Generic)** subscriptions and **QoS (Quality of Service)** configuration.

---
source code file:

```https://github.com/ros/ros_comm/blob/noetic-devel```


## 2. Design Philosophy of Gaps (TODOs)

The gaps are designed as "hollowed-out" function bodies. This forces the model to reason about the local implementation while ensuring the code remains consistent with the surrounding ROS 2 architecture.

### A. The "Timer & Parameter" Gap (`talker.py`)
* **The Task**: Replace the legacy `while not rospy.is_shutdown()` sleep loop.
* **The Logic**: In ROS 2, nodes must avoid blocking the main thread. The model is expected to implement a `create_timer` callback. It must also utilize the **Parameter API** to fetch the `topic_name`, reflecting the ROS 2 "Configuration over Coding" philosophy.

### B. The "Generic Subscription" Gap (`recorder.cpp`)
* **The Task**: Implement a subscriber that can record any topic without knowing its data type at compile time.
* **The Logic**: This tests if the model knows about `rclcpp::GenericSubscription`. It also validates **QoS Awareness**: the model must choose `SensorDataQoS` or `BestEffort` for high-bandwidth topics (like LIDAR/Images) to prevent network congestion.

### C. The "Temporal Integrity" Gap (`recorder.cpp`)
* **The Task**: Timestamp the recorded messages.
* **The Logic**: LLMs often hallucinate by using `std::chrono` or system time. The model must use `node_->now()` to ensure the data is timestamped according to the **ROS Domain Clock**, which is vital for accurate bag playback.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle utilizes regex-based "Intent Detection" to verify that the model implemented the correct architectural patterns rather than just generating compilable syntax.

### Evaluation Criteria

| Test Case | Intent | Expected Code Outcome (Example) |
| :--- | :--- | :--- |
| **Generic Intent** | Type-agnostic recording | `node->create_generic_subscription(topic, "rmw_serialized_message", ...)` |
| **QoS Awareness** | Handle sensor streams | `rclcpp::QoS(10).best_effort()` or `rclcpp::SensorDataQoS()` |
| **Clock Source** | System-wide sync | `node_->now()` or `this->now()` |
| **Parameter Intent**| Dynamic configuration | `self.declare_parameter` & `self.get_parameter` |
| **Timer Intent** | Event-driven execution | `self.create_timer(0.1, self.timer_callback)` |
| **Cleanliness** | Zero legacy leakage | **Fail** if `import rospy` or `ros::NodeHandle` is found. |
