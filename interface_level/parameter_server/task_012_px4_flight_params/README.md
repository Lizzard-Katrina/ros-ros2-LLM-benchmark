# Task 012: PX4 Dynamic Flight Parameter Integration (Python)

## 1. Brief Description
This task focuses on the "Parameterization" of a PX4 Offboard Control node. In typical robotics workflows, mission-critical values—such as takeoff altitude, velocity limits, and target headings—should never be hardcoded. This task requires the developer to migrate static variables to the ROS 2 Parameter Server using `rclpy`, and correctly map these dynamic values to the specific array-based message structures used by PX4 (`px4_msgs`).
---
source code
```https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py```

## 2. Excavation Strategy: The "State-to-Message" Flow
The excavation is designed to test whether the model can maintain a consistent data flow across different class methods without "hand-holding" TODOs.

* **Macro-Excavation of `__init__`**: We remove the entire block responsible for parameter declaration and retrieval. The model must autonomously use `self.declare_parameter` and `self.get_parameter(...).value` to initialize the node's state.
* **Logical Bridge in Publishers**: We excavate the population of the `TrajectorySetpoint` message. This forces the model to correctly bridge the class attributes (stored parameters) with the actual ROS 2 middleware calls.
* **Interface Integrity**: By removing the message assignment logic, we test if the model knows the specific field definitions of PX4 messages (e.g., using a `position` array instead of individual `x, y, z` fields).

## 3. Oracle Testcase Design & Expected Outcomes

The Oracle suite uses a comment-stripping pre-processor to ensure it only validates functional logic.

| Testcase | Design Intent | Expected Outcome / Passing Criteria |
| :--- | :--- | :--- |
| **Parameter Declaration** | Ensures parameters are registered with the ROS 2 core. | Code must contain `self.declare_parameter` for both `'takeoff_height'` and `'target_yaw'`. |
| **Value Retrieval** | Validates the transition from Parameter Server to Class State. | Must use `self.get_parameter(...).value` (or equivalent) to assign values to `self` attributes. |
| **Field Integrity (PX4)** | **CRITICAL:** Checks for correct PX4 message structure. | **Must NOT** use `msg.x`, `msg.y`, or `msg.z`. **MUST** use the `msg.position` array for 3D coordinates. |
| **Dynamic Yaw Mapping** | Ensures the heading is not hardcoded in the loop. | `msg.yaw` must be assigned from a class attribute (e.g., `self.target_yaw`) rather than the raw float `1.57079`. |
| **Timestamp Conversion** | Validates domain-specific data requirements. | Must convert ROS 2 nanoseconds to PX4 microseconds using `nanoseconds / 1000`. |
| **Safety Logging** | Tests for production-ready "Defensive Programming." | Use of `self.get_logger().info()` to print the loaded parameters during the constructor phase. |
| **Anti-Hardcoding Check** | Scans for "lazy" migration artifacts. | Failure if raw numbers like `-5.0` or `1.57079` are found in assignment statements outside of the default parameter declarations. |

## 4. Why This Task Matters
Failing this task usually results in one of three "Real-World" disasters:
1.  **Node Crash**: Using `msg.x` instead of `msg.position` causes an `AttributeError` at runtime.
2.  **Failsafe Trigger**: Incorrect microsecond timestamping causes the PX4 EKF to reject setpoints, triggering a forced landing.
3.  **Parameter Locking**: Failing to use `declare_parameter` prevents the node from being configured via external YAML files, breaking CI/CD pipelines.
