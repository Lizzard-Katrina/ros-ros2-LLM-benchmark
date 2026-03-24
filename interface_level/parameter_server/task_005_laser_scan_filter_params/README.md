# Task 005: Laser Scan Filter Pipeline Integration

## 🎯 Objective
Migrate a ROS 1 Laser Scan filter node to ROS 2, focusing on the integration of `filters::FilterChain` and `tf2_ros::MessageFilter`. This task tests the LLM's ability to handle complex ROS 2 component lifecycles, lifecycle interfaces, and modern C++ synchronization primitives.
---
source code:
```https://github.com/ros-perception/laser_filters/blob/rolling/src/generic_laser_filter_node.cpp```

## 🏗️ Architectural Requirements
The node must bridge several ROS 2 components into a functional pipeline:
1.  **Parameter Transparency**: Use `NodeOptions` to allow the filter chain to read nested plugin parameters from YAML.
2.  **Interface Binding**: Correctily bind the `FilterChain` to the node's logging and parameter interfaces.
3.  **Temporal Synchronization**: Setup a `tf2_ros::MessageFilter` with specific time tolerance and callback binding.
4.  **Modern Communication**: Utilize `SensorDataQoS` for high-frequency laser streams.
5.  **Legacy Management**: Implement a deprecation warning system using wall timers.

## 🕳️ The "Hole" (Code to be generated)
The entire logic inside the `GenericLaserScanFilterNode` constructor and the `main` node initialization is removed. The model must:
* Configure `rclcpp::NodeOptions`.
* Initialize the `FilterChain` with correct interface pointers (ordering is critical for C++ compilation).
* Setup the `MessageFilter` with `std::bind` and `std::chrono` durations.
* Implement a `create_wall_timer` for deprecation logs.

## ✅ Evaluation: Oracle Test Cases
The Oracle tests use regex-based semantic matching to validate the implementation without execution.

| Test Case | Validates | Success Criteria |
| :--- | :--- | :--- |
| `test_filter_chain_interface_usage` | **API Signature** | Matches `configure(prefix, logging, params)`. **Strict order required.** |
| `test_tf_filter_binding` | **Synchronization** | Proper use of `setTolerance` (30ms) and `registerCallback`. |
| `test_qos_and_topics` | **Communication** | Presence of `output` topic and use of `SensorDataQoS`. |
| `test_deprecation_timer` | **Lifecycle** | 5s timer exists and logs the specific migration warning. |
| `test_chrono_and_bind_usage` | **Style Constraints** | Usage of `std::bind` and `std::chrono` as per the prompt. |
| `test_no_ros1_symbols` | **Migration Quality** | Absence of `ros::NodeHandle` or other legacy symbols. |

## ⚠️ Common Failure Modes
* **Signature Swapping**: (As seen in testing) Swapping the logging and parameter interfaces in `filter_chain_.configure()`. This causes C++ compilation failure.
* **QoS Mismatch**: Using default QoS (Reliable) instead of `SensorDataQoS` (Best Effort) for sensor data.
* **Chrono Neglect**: Using raw integers (e.g., `0.03`) instead of `30ms` or `std::chrono::milliseconds(30)`.
