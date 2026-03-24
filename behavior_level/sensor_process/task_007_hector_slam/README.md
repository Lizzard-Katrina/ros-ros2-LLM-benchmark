# Task 010: Hector Mapping Core SLAM Pipeline Migration (ROS 2)

## 1. Brief Description
This task involves migrating the central processing unit of Hector SLAM—the `scanCallback` in `HectorMappingRos.cpp`—from ROS 1 to ROS 2 Humble. The node is responsible for taking raw `LaserScan` data, resolving coordinate transforms, updating the occupancy grid map, and broadcasting the robot's localization.

Key technical challenges:
* **Complex TF2 Chain**: Calculating the `map -> odom` transform by combining the SLAM result (`map -> base`) with the odometry offset (`odom -> base`).
* **Time System Migration**: Moving from `ros::Time` to `rclcpp::Time` and handling `builtin_interfaces` with standard TF2 conversion utilities.
* **QoS Requirements**: Ensuring SLAM data (Maps and Poses) are published with `TransientLocal` durability so late-joining nodes (like RViz2) can receive the latest state.
* **Memory Efficiency**: Implementing zero-copy patterns using `std::move` for large message types.

---
source code file

```https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/noetic-devel/hector_mapping/src/HectorMappingRos.cpp```
## 2. Hollowing Strategy
We hollow out the entire logic inside `HectorMappingRos::scanCallback`. To ensure the LLM follows modern ROS 2 best practices and satisfies the Oracle tests, the following **MANDATORY CONSTRAINTS** are added to the TODO:

* **Syntax Strictness**: Forcing pointer-style access for `tf_buffer_` to prevent common ref vs. pointer ambiguity.
* **Mathematical Path**: Requiring the use of `.inverse()` for the transform chain to verify the developer understands the coordinate geometry ($T_{map \to odom} = T_{map \to base} \times T_{base \to odom}$).
* **Standardized API**: Explicitly banning manual `chrono` nanosecond arithmetic in favor of `tf2_ros::fromMsg`.
* **State Management**: Requiring `TransientLocal` QoS durability for persistent sensor data.

---

## 3. Oracle Test Design & Expected Outcomes

| Test Case | Design Intent | Expected Outcome |
| :--- | :--- | :--- |
| **`test_tf2_lookup_syntax`** | Ensures consistent API usage for the TF2 buffer. | Matches `tf_buffer_->lookupTransform`. |
| **`test_inverse_transform_usage`** | Validates the mathematical derivation of the transform chain. | Code contains `.inverse()` to resolve the odom offset. |
| **`test_qos_durability_policy`** | Ensures the SLAM data is visible to late-joining subscribers. | Contains `TransientLocal` or `transient_local` keywords. |
| **`test_standard_tf2_time_api`** | Enforces the use of standard ROS 2 time conversion utilities. | Contains `tf2_ros::fromMsg` and NO manual `chrono::nanoseconds`. |
| **`test_ros2_logging_and_time`** | Verifies migration of core Node logging and clock APIs. | Matches `RCLCPP_`, `this->get_logger()`, and `this->now()`. |
| **`test_timestamp_preservation`** | Prevents temporal drift by ensuring sensor-to-output sync. | Matches `header.stamp = scan->header.stamp`. |
| **`test_zero_ros1_leakage`** | Scans for deprecated or non-migrated ROS 1/Boost symbols. | Zero occurrences of `ros::Time`, `tf::Transform`, or `ros::ok()`. |
