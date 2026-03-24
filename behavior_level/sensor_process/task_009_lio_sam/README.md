# Task 009: LIO-SAM Map Optimization Migration (ROS 2)

## 1. Brief Description
This task involves migrating the `mapOptimization.cpp` node from the LIO-SAM (Lidar Inertial Odometry via Smoothing and Mapping) framework to ROS 2 Humble. This node is the central hub for factor graph optimization (GTSAM), global map maintenance, and broadcasting the critical `map -> odom` coordinate transform.

The migration requires transitioning from the synchronous, global-pointer-heavy model of ROS 1 to the asynchronous, multi-threaded, and Node-based execution model of ROS 2.
---
source code fiile
```https://github.com/TixiaoShan/LIO-SAM/blob/master/src/mapOptmization.cpp#L355```

## 2. Hollowing Strategy
To focus on ROS 2 middleware and concurrency rather than complex SLAM mathematics, we hollow out the following functional interfaces:

* **`void publishOdometry()`**: The primary output for spatial localization. Candidates must implement the TF2 broadcasting logic, ensuring strict temporal alignment between the optimized pose and the original sensor data.
* **`void saveMapService(...)`**: The administrative interface for map management. Candidates must implement the ROS 2 asynchronous service pattern, including thread-safe access to shared keyframe buffers.

## 3. Oracle Testcase Design

### I. Timestamp Synchronization (`test_tf_timestamp_synchronization`)
* **Design Intent**: In SLAM, the transform must represent the state of the world at the exact moment sensor data was captured. Using `this->now()` (current node time) introduces "time-skew," causing point clouds to jitter or drift.
* **Expected Outcome**: The code must assign `timeLaserInfoStamp` to `header.stamp`. Use of `now()` or `get_clock()` in the TF assignment block results in a failure.

### II. Real-time Concurrency (`test_callback_group_initialization`)
* **Design Intent**: Map optimization and I/O are CPU-intensive. In ROS 2, without a dedicated `CallbackGroup`, these tasks will block the real-time reception of incoming lidar scans.
* **Expected Outcome**: The node must show evidence of `create_callback_group` or `callback_group_` usage, indicating the node is prepared for a `MultiThreadedExecutor`.

### III. Data Race Protection (`test_mutex_locking_in_service`)
* **Design Intent**: The `saveMapService` reads from global keyframe containers while the optimization thread may be modifying them. 
* **Expected Outcome**: Presence of `std::lock_guard<std::mutex>` within the service callback to protect shared data structures.

### IV. Service API Standards (`test_service_parameter_shared_ptrs`)
* **Design Intent**: ROS 2 services require `std::shared_ptr` for request/response objects and follow a specific signature.
* **Expected Outcome**: The callback must strictly use `(const std::shared_ptr<...Request>, std::shared_ptr<...Response>)`.

### V. Response Logic (`test_service_response_success_set`)
* **Design Intent**: Verifies that the service communicates its execution status back to the client.
* **Expected Outcome**: Explicit assignment to `res->success` (true/false).

### VI. Clean Migration (`test_no_legacy_ros1_symbols`)
* **Design Intent**: Ensures the code is fully migrated and does not rely on ROS 1 compatibility shims or legacy headers.
* **Expected Outcome**: Zero instances of `ros::Time`, `ros::ok()`, `ROS_INFO`, or `tf::`.
