# Task 009: OdomTracker Realtime-Safe Migration

## Context
You are migrating the `OdomTracker` component from a ROS 1 SMACC state machine to **ROS 2 (SMACC2)**. This component is performance-critical as it records the robot's trajectory in real-time and provides path history for backward navigation.
---
source code:
```https://github.com/robosoft-ai/SMACC/blob/noetic-devel/smacc_client_library/move_base_z_client/move_base_z_client_plugin/src/components/odom_tracker/odom_tracker.cpp```

## Objectives
1.  **Node Refactoring**: Transform the class to inherit from `rclcpp::Node`.
2.  **Parameter Management**: Replace legacy `ros::NodeHandle::getParam` with the ROS 2 `declare_parameter` and `get_parameter` lifecycle.
3.  **Realtime Safety**: Migrate `realtime_tools::RealtimePublisher` to its ROS 2 equivalent, which requires explicit access to the `NodeBaseInterface`.
4.  **Timing & Logging**: Synchronize timestamps using the Node's clock (`this->now()`) and transition all logging to `rclcpp` macros.

## Critical Constraints (Oracle Compliance)
To pass the automated validation, your implementation **MUST** strictly follow these patterns:
* **Inheritance**: Use `this->` to access node methods (e.g., `this->declare_parameter`, `this->get_logger()`).
* **Initialization**: `RealtimePublisher` must be initialized with `this->get_node_base_interface()`.
* **Concurrency**: All path publishing must use the `trylock()` and `unlockAndPublish()` pattern to ensure the odometry callback remains non-blocking.
* **Namespaces**: All ROS messages must include the `msg::` middle namespace (e.g., `nav_msgs::msg::Path`).

## File Structure
* `src/odom_tracker.cpp`: The file containing the HOLES to be filled.
* `test/test_oracle_ros2.py`: The regex-based semantic validator.

## Evaluation
The Oracle will fail if it detects:
* Legacy ROS 1 symbols (`ros::NodeHandle`, `ros::Time::now`).
* Incorrect `RealtimePublisher` constructor signatures (missing `NodeBaseInterface`).
* Implicit parameter usage without prior declaration.
* Use of `boost::bind` instead of modern `std::bind`.
