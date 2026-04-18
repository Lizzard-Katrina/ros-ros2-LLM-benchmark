# Task 005: Allegro Hand V5 IMU ROS 2 Migration

## 1. Brief Description
This task requires migrating a high-fidelity IMU Gazebo plugin from a legacy ROS 1/PX4-based architecture to a modern **ROS 2 Humble** environment. The plugin simulates an IMU (Inertial Measurement Unit) mounted on an Allegro Hand V5 link. It features a sophisticated stochastic noise model, including **white noise scaling** and **Gauss-Markov bias random walk**. The objective is to ensure that while the middleware changes (ROS 1 to ROS 2), the underlying physics and communication safety remain intact.

---
source code file:
```https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main```

## 2. Rationale for Code Gaps (Holes)

### A. CMakeLists.txt (Build System)
* **Gap:** The `find_package` and dependency linking sections.
* **Reason:** This tests the developer's ability to transition from `catkin` to `ament_cmake`. Specifically, replacing `roscpp` with `rclcpp` and using `ament_target_dependencies` to handle the new ROS 2 message generation and linking workflow.

### B. gazebo_imu_plugin.cpp (Physics & API)
* **Gap 1: Noise Scaling Logic (`addNoise`)**
    * **Reason:** In discrete-time physics simulation, white noise must be scaled by $1/\sqrt{dt}$. Forgetting this is a common error that makes sensor noise dependent on the simulation frequency.
* **Gap 2: Gravity Compensation (`OnUpdate`)**
    * **Reason:** IMU sensors measure specific force. Correct implementation requires rotating the gravity vector into the local body frame (`RotateVectorReverse`) before subtraction.
* **Gap 3: ROS 2 Publisher API (`Load` & `OnUpdate`)**
    * **Reason:** Validates the transition from `ros::Publisher` and Protobuf setters to `rclcpp::Publisher` and direct member assignment in ROS 2 messages.

### C. msgbuffer.h (Low-level Communication)
* **Gap:** Buffer size definitions and memory copy logic.
* **Reason:** Ensures the developer maintains the low-level serialization integrity required for MAVLink-style communication, emphasizing memory safety via `assert` and proper use of `memcpy`.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle Tests use static analysis (Regex) to verify the implementation. Below is the design logic for each:

| Test Case | Design Logic | Expected Outcome |
| :--- | :--- | :--- |
| `test_cmake_ros2_syntax` | Searches for `rclcpp` and `ament_package()`. | `PASS` if the build system is correctly migrated to ROS 2. |
| `test_imu_physics_frequency_scaling` | Looks for the $1/\sqrt{dt}$ (or `1/sqrt(dt)`) term in noise calculations. | `PASS` if white noise is correctly frequency-invariant. |
| `test_gravity_rotation_logic` | Checks for `RotateVectorReverse` applied to gravity. | `PASS` if acceleration is measured in the Body frame. |
| `test_ros2_node_and_publisher_api` | Matches `create_publisher` and `node_->now()`. | `PASS` if the plugin uses native ROS 2 node handles. |
| `test_ros2_message_field_access` | Checks for `.header.frame_id` and `.linear_acceleration.x`. | `PASS` if ROS 2 struct access is used instead of Protobuf setters. |
| `test_no_ros1_remnants` | Blacklists symbols like `ros::NodeHandle` or `ros/ros.h`. | `PASS` if the code is 100% clean of ROS 1 legacy symbols. |
| `test_msgbuffer_logic_preservation` | Verifies `mavlink_msg_to_send_buffer` and `MAX_SIZE`. | `PASS` if the low-level buffer remains functionally intact. |
| `test_msgbuffer_safety_assertions` | Checks for `assert(len < MAX_SIZE)`. | `PASS` if bounds checking is implemented to prevent overflows. |

