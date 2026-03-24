# Task 008A: Robot Control Mux Ultra-Strict Migration (C++)

## 1. Brief Description
This task involves migrating the `RobotControlMux` node from ROS 1 to ROS 2. This node is a critical "traffic controller" that switches a robot's velocity commands between autonomous navigation and manual teleoperation. 
---
source code file:
```https://github.com/MarcoStb1993/robot_statemachine/blob/master/rsm_core/src/RobotControlMux.cpp#L78```

## 2. Hollowing Strategy
- **Hole A (Node Infrastructure)**: Removes the constructor. The model must implement the ROS 2 Node pattern, including **mandatory parameter declaration**, QoS object instantiation, and subscription/publisher setup.
- **Hole B (Service Logic)**: Removes the `setOperationMode` service callback. This tests the specific ROS 2 `std::shared_ptr` service signature and pointer-based member access.

## 3. Oracle Test Design (Ultra-Strict)
The test suite is designed to catch "lazy" migrations where a model might hardcode values instead of migrating the configuration interface.

| Test Case | Requirement | Failure Trigger |
| :--- | :--- | :--- |
| `test_parameter_lifecycle` | Declare at least 3 parameters. | Fails if topic names or timeouts are hardcoded as strings. |
| `test_ros2_qos_instantiation` | Explicit `rclcpp::QoS` object. | Fails if only a '10' (integer) is passed to publishers. |
| `test_ros2_service_signature` | Exact `request->` / `response->` naming. | Fails if the model ignores the [STYLE] guide or uses `req/res`. |
| `test_no_ros1_remnants` | Zero ROS 1 symbols. | Fails if `NodeHandle` or `ros::Timer` appears in code/comments. |
| `test_timer_chrono_strict` | `create_wall_timer` + `std::chrono`. | Fails if legacy timing or integer durations are used. |

## 4. Key Challenge
The model must not only fix the syntax but also preserve the **programmability** of the node by declaring parameters for all configurable topics, ensuring the ROS 2 version remains as flexible as the ROS 1 original.
