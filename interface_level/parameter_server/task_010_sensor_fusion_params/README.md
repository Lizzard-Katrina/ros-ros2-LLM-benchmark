# Task 010: Migration of Complex Sensor Parameter Loading in robot_localization

## 1. Brief Description

This task focuses on one of the most parameter-intensive and architecturally complex packages in the ROS ecosystem: `robot_localization`. The goal is to migrate the `RosFilter<T>::loadParams()` function from ROS 1 to ROS 2. 

This function is the central hub for discovering and configuring all sensor inputs (Odometry, Pose, Twist, IMU, and Acceleration). The core challenge involves implementing the logic to:
- **Dynamically Discover Sensors**: Iterate through indexed sensor types (e.g., `odom0`, `odom1`, `imu0`...).
- **Handle ROS 2 Parameter Lifecycle**: Implement the mandatory `declare_parameter` before `get_parameter` pattern.
- **Parse Complex Matrices**: Retrieve and validate 15-element boolean "Update Vectors" that map sensor data to the filter's state estimate.
- **Migrate Logging**: Transition from legacy `ROS_INFO/WARN` macros to the node-based `RCLCPP_INFO/WARN` system.
---
source code file
```https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/src/ros_filter.cpp```

## 2. Reasons for Excavation

The `loadParams` function in `src/ros_filter.cpp` was selected for excavation for several critical reasons:

* **Paradigm Shift in Parameter Access**: ROS 1 allows implicit parameter fetching via `nh.getParam`. ROS 2 requires explicit declaration. This is a primary friction point in migration that tests the LLM's understanding of the `rclcpp` API.
* **Domain-Specific Constraints**: `robot_localization` relies on a specific mathematical model involving a 15-dimensional state vector. An LLM must demonstrate "Robotics Domain Awareness" by enforcing the 15-element size constraint on configuration vectors, distinguishing a specialized robotics AI from a general-purpose coding assistant.
* **Logic Reconstruction Complexity**: The original code uses nested loops and dynamic string concatenation to build parameter keys. Excavating this requires the LLM to prove it can maintain functional equivalence while adopting modern ROS 2 coding standards.

## 3. Oracle Test Design and Expected Outcomes

The Oracle tests use Python's `pytest` framework with semantic regex matching to ensure the solution is functionally correct regardless of minor stylistic differences.

### T1: ROS 2 Parameter Declaration
* **Design Intent**: Verifies that the LLM understands the requirement to declare parameters before use.
* **Expected Outcome**: Presence of `this->declare_parameter<std::vector<bool>>` or equivalent typed declaration calls.

### T2: Sensor Config Naming Logic
* **Design Intent**: Ensures the code correctly constructs parameter names following the `<type><index>_config` convention (e.g., `odom0_config`).
* **Expected Outcome**: Regex match for logic concatenating a prefix (literal or variable), an index, and the `"_config"` suffix.

### T3: Vector 15-Element Validation
* **Design Intent**: Checks for the enforcement of the package's most critical mathematical constraint.
* **Expected Outcome**: The code must contain a logic branch checking if the retrieved vector size equals `15`, typically followed by a size-mismatch warning.

### T4: Legacy Syntax Removal
* **Design Intent**: Guard against "hallucinated" ROS 1 code or lazy porting.
* **Expected Outcome**: Absence of `ros::NodeHandle`, `nh.getParam`, or `ros::init` keywords.

### T5: Logger Context Migration
* **Design Intent**: Verifies the transition from global logging macros to node-bound loggers.
* **Expected Outcome**: `RCLCPP_` macros must correctly utilize `this->get_logger()` as the first argument.

### T6: Parameter Retrieval Integrity
* **Design Intent**: Confirms that the code actually fetches values into the filter's internal data structures.
* **Expected Outcome**: Successful matching of `get_parameter` calls that map the declared keys to local or member variables.
