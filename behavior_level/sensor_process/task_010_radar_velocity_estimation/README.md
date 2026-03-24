# Task 11: Radar Ego Velocity Estimator ROS 2 Migration

## 1. Brief Description
The core objective of this task is to evaluate a Large Language Model's (LLM) capability to migrate a **ROS 1 sensor processing node** to **ROS 2 (Humble/Foxy)**. 
The target file, `radar_ego_velocity_estimator_ros.cpp`, is responsible for receiving Radar point cloud data (`PointCloud2`) and estimating the vehicle's ego-velocity based on whether an external trigger signal (`trigger_stamp`) is present.

This task challenges the model's understanding of several critical ROS 2 shifts:
* **Memory Management**: Transitioning from Boost-based pointers (`ConstPtr`) to C++11 smart pointers (`SharedPtr`).
* **Clock System**: Moving from the global `ros::Time` to the node-associated `rclcpp::Time`.
* **Concurrency Control**: Maintaining thread safety via `std::mutex` within the ROS 2 multi-threaded executor paradigm.
* **Logging System**: Migrating from global macros to node-contextual `RCLCPP` macros.

---
source code
```https://github.com/christopherdoer/reve/blob/master/radar_ego_velocity_estimator/src/radar_ego_velocity_estimator_ros.cpp```

## 2. Hollowing Strategy
We utilize a **"High-Level Intent + Strict Naming Contract"** strategy for the hollowing process to ensure the benchmark measures architectural reasoning rather than simple line-by-line translation.

* **Hollowed Range**: The entire function body of `callbackRadarScan`.
* **Constraint Setting**:
    * **Naming Contract**: The parameter name is strictly forced to `radar_scan_msg` to ensure precise pattern matching during Oracle Testing.
    * **Logic Integrity**: The model is explicitly warned to preserve all functional branches, specifically the `run_without_trigger` conditional logic.
* **Migration Trap**: No explicit hint is given regarding the `SharedPtr` syntax, testing if the model can autonomously identify and fix ROS 1 legacy `ConstPtr` types in the signature.

---

## 3. Oracle Test Design & Expected Outcomes

Each test case is designed to capture specific "semantic drifts" during the migration process via regex-based pattern matching.

### Test Case 1: `test_ros2_logging_migration`
* **Design Goal**: Checks if the model realizes ROS 2 logs must pass through `get_logger()`.
* **Expected Outcome**: Code must contain `RCLCPP_WARN(this->get_logger(), ...)` or similar node-based macros.

### Test Case 2: `test_shared_ptr_dereferencing`
* **Design Goal**: Validates the handling of ROS 2 message pointers and variable naming compliance.
* **Expected Outcome**: A correct call to `processRadarData` using the mandatory variable name `radar_scan_msg`.

### Test Case 3: `test_ros2_clock_usage`
* **Design Goal**: Verifies the move away from the `ros::Time::now()` singleton.
* **Expected Outcome**: Implementation of `this->now()` or `this->get_clock()->now()`.

### Test Case 4: `test_mutex_lock_guard`
* **Design Goal**: Evaluates thread-safety awareness in the ROS 2 asynchronous callback environment.
* **Expected Outcome**: Proper use of `std::lock_guard<std::mutex>` with the member variable `mutex_`.

### Test Case 5: `test_trigger_reset_logic_strict`
* **Design Goal**: **Critical Logic Check**. Verifies the model understands the state machine: the `trigger_stamp` must be cleared after consumption to prevent stale data.
* **Expected Outcome**: An assignment such as `trigger_stamp = rclcpp::Time();` or equivalent reset logic.

### Test Case 6: `test_zero_timestamp_handling`
* **Design Goal**: Checks for the migration of edge-case handling (zero-valued timestamps from radar drivers).
* **Expected Outcome**: Explicit use of the `.sec == 0` syntax as requested in the task constraints.

### Test Case 7: `test_branch_preservation`
* **Design Goal**: Prevents the LLM from over-simplifying code by deleting the `run_without_trigger` mode.
* **Expected Outcome**: Presence of conditional logic handling both trigger and non-trigger modes.

### Test Case 8: `test_no_ros1_namespaces`
* **Design Goal**: The final "safety gate" ensuring no legacy ROS 1 symbols persist.
* **Expected Outcome**: Absence of `ros::NodeHandle`, `ros::Subscriber`, or `ros::Time`.
