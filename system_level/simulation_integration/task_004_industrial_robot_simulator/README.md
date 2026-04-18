# Task 004: Industrial Robot Simulator - ROS 2 Migration & Integration

## 1. Brief Description
This task involves migrating and integrating a robot simulation system using the **ROS 2** version of the ROS-Industrial (ROS-I) framework. You are required to finalize a three-component system:
1.  **A State Feedback Node**: Bridging the robot's TCP state to ROS 2 topics.
2.  **A Trajectory Interface**: Handling the mapping between ROS 2 trajectory messages and industrial protocols.
3.  **A System Launch Configuration**: Defining the global joint topology for the 6-DOF manipulator.
---
source code file:
```https://github.com/ros-industrial/industrial_core/blob/melodic-devel/industrial_robot_simulator```

## 2. Hollowing Logic

### A. Lifecycle Initialization (`generic_robot_state_node.cpp`)
* **Reason**: In ROS 2, simply spinning a node (`rclcpp::spin`) is insufficient for industrial clients. The `RobotStateInterface` contains a specialized TCP/IP handler that must be explicitly started via `init()`. Without this, the node remains a "silent shell" that never attempts to connect to the robot/simulator.

### B. Dynamic Parameter Mapping (`joint_trajectory_interface.cpp`)
* **Reason**: Industrial robots often have different joint naming conventions than the URDF. We hollowed out the `getJointNames` retrieval logic. Developers must prove they can bridge the **Parameter Server** (fetching `controller_joint_names`) with the **C++ Interface** to ensure commands reach the correct physical joints.

### C. ROS 2 Launch Topology (`robot_interface_simulator.launch.py`)
* **Reason**: Migration is not just about code; it's about the ecosystem. We hollowed the parameter declaration in the launch file. This tests the understanding of ROS 2's new parameter declaration requirements and the shift from ROS 1 XML `rosparam` to ROS 2's specific Python-based parameter mapping.

## 3. Testcase Design & Expected Outcomes

| Testcase | Design Intent | Expected Outcome (Pass) |
| :--- | :--- | :--- |
| `test_ros2_state_node_lifecycle` | Validates Smart Pointer instantiation and the explicit trigger of the TCP stack. | Code must use `std::make_shared` AND call `node->init()` before `rclcpp::spin()`. |
| `test_ros2_trajectory_param_logic` | Ensures ROS 2 parameter "declaration" (mandatory in ROS 2) is implemented. | The code must contain `declare_parameter` for `controller_joint_names` and use `getJointNames`. |
| `test_launch_system_migration_check` | Strictly forbids legacy ROS 1 XML formats to ensure a clean migration. | The launch file must NOT contain `<launch>` tags; it must follow ROS 2 Python/YAML syntax. |
| `test_no_ros1_symbols` | Scans for "Ghost Symbols" (leftover ROS 1 code). | Zero occurrences of `ros::init`, `ros::NodeHandle`, or `ros::spin()`. |

## 4. Expected System Behavior
Once all three files are correctly implemented:
1.  **Handshake**: The State Node will log a successful connection to the simulator's TCP port.
2.  **Mapping**: The Trajectory Interface will successfully validate a 6-joint trajectory against the URDF.
3.  **Simulation**: Moving the robot will result in visible motion in the `industrial_robot_simulator`, with joint state feedback flowing back to the ROS 2 `/joint_states` topic.
