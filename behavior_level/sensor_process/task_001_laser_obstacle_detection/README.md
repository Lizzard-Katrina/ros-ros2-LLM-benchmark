# Task 001: Laser Filter Pipeline - Behavioral Orchestration

## 1. Brief Description
This task initiates the **Behavior Level (Sensor → Processing → Output)** track. It focuses on the core data pipeline of the `scan_to_scan_filter_chain` node. The objective is to wire the ROS 2 middleware to an algorithmic filter chain. This involves subscribing to raw Lidar data, synchronizing it with the robot's coordinate transforms (TF2), processing it, and publishing the "cleaned" output.

In the Behavior Level, success is measured by the **integrity of the data flow** and the **robustness of the synchronization**, ensuring that downstream navigation and mapping nodes receive high-quality, timely data.

---
source code
```https://github.com/ros-perception/laser_filters/blob/rolling/src/scan_to_scan_filter_chain.cpp```

## 2. Excavation Strategy: The "Black-Box" Pipeline
The excavation removes the "connective tissue" of the node, forcing the model to architect the data flow from scratch. We target two critical functional areas:

* **The Pipeline Wiring (Constructor)**: The logic that determines how the node listens to the world. The model must decide between a standard `Subscription` and a **TF-Synchronized Message Filter** based on the configuration of a target frame.
* **The Processing Loop (Callback)**: The transition logic from an input message pointer to a processed result. The model must autonomously implement the filter update and the conditional publication gate.

**Key Challenge**: The model is provided with the *functional goal* (e.g., "Implement TF-aware sync") but not the *implementation steps*. It must choose the correct ROS 2 patterns (smart pointers, `std::bind`, QoS settings) without explicit step-by-step instructions.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle suite for this task is "Behavior-Aware," focusing on architectural pitfalls such as scope errors, resource lifecycle, and API misuse.

| Test Case | Design / Intent | Expected Outcome (To Pass) |
| :--- | :--- | :--- |
| **TF Listener Persistence** | Detects **Scope Errors**. Many models mistakenly create the TF listener as a local variable, which is destroyed when the constructor exits, killing the data stream. | The TF Listener must be assigned to a **class member** (e.g., `this->tf_`), not a local `auto` variable. |
| **Filter Execution Logic** | Verifies that the "Processing" step actually happens. Supports both in-place and 2-argument update signatures. | Must contain a call to `filter_chain_.update(...)` within the callback. |
| **Conditional Sync Branching** | Checks if the model understands that TF-Sync is an optional behavioral branch. | Must contain an `if-else` structure checking if `tf_message_filter_target_frame_` is empty. |
| **No Lifecycle Hallucination** | Prevents the AI from "hallucinating" `LifecycleNode` methods (like `on_activate`) in a standard `rclcpp::Node`. | The code must **not** contain `on_activate` or `set_on_new_subscription_callback`. |
| **Perception Safety Gate** | Ensures "Junk Data" isn't published. If the filter fails, the data flow must be halted. | The `publish()` call must be nested inside an `if` block that validates the return value of the filter update. |
| **QoS Consistency** | Ensures sensor data is handled with the correct priority and reliability settings. | Must explicitly use `rclcpp::SensorDataQoS()` for the subscription. |

---

## 4. Engineering Impact
A successful implementation proves the model can handle **Active Perception Pipelines**. By passing these tests, the model demonstrates it can manage three distinct ROS 2 components (Subscriber, Publisher, and TF-Filter) as a single, cohesive, and thread-safe behavioral unit. This is the foundation for all real-time robot processing tasks.
