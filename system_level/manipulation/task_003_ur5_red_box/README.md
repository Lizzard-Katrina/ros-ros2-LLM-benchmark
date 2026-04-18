# Task 003: UR5 Red Box - Communication & Build Migration

## 1. Brief Description
This task focuses on the complete migration of a robotic manipulation control interface from **ROS 1 (Noetic/Kinetic)** to **ROS 2 Humble**. 

The scenario involves a UR5 robot tasked with picking a red box. The core objective is to migrate the custom service-client communication layer and the underlying build system. This requires transitioning from the synchronous `rospy` and `catkin` environment to the asynchronous, executor-based architecture of `rclpy` and the `ament_cmake` build system, while ensuring custom service interfaces (`SetJointStates.srv`) are correctly generated and accessible.

---
source code direcory:
```https://github.com/kkumpa/ros-robotic-arm/blob/master/robotic_arm_algorithms```


## 2. Design Logic for Implementation (Holes)

### Hole 1: Service Server Migration (`set_joint_states_service.py`)
* **Logic:** ROS 2 service callbacks use a pre-instantiated `response` object. The execution must be non-blocking to the executor.
* **Requirement:** Refactor the ROS 1 callback to accept `(request, response)` arguments. The implementation must populate the `response` fields and return the object explicitly. The node must be initialized as a standard ROS 2 `Node` class.

### Hole 2: Asynchronous Client Logic (`set_joint_states_client.py`)
* **Logic:** Calling services synchronously in a single-threaded ROS 2 executor often leads to deadlocks.
* **Requirement:** Implement a client that uses `call_async()`. The control flow must use `rclpy.spin_until_future_complete()` or a manual `spin_once()` loop to wait for the `Future` result without freezing the process.

### Hole 3: Modern Build System (`CMakeLists.txt`)
* **Logic:** ROS 2 replaces `message_generation` with the `rosidl` pipeline.
* **Requirement:** Implement `rosidl_generate_interfaces` to compile the `.srv` file. All install targets must be updated to the ROS 2 standard `lib/${PROJECT_NAME}` directory to ensure scripts are discoverable by `ros2 run`.

### Hole 4: Package Manifest (`package.xml`)
* **Logic:** ROS 2 requires Format 3 for advanced dependency grouping.
* **Requirement:** Update the package format to `3`. Crucially, add the package to the `rosidl_interface_packages` member group to allow Python-based service discovery.

---

## 3. Oracle Test Design and Expected Outcomes

| Test Case | Design Intent | Expected Outcome (Pass Criteria) |
| :--- | :--- | :--- |
| **test_service_server_migration** | Validates the structural integrity of the Service provider. | **Pass** if the callback uses `(request, response)` and returns `response`. |
| **test_service_client_async** | Ensures the client won't deadlock in a ROS 2 environment. | **Pass** if `call_async` is used and the `Future` object is properly handled. |
| **test_cmakelists_rosidl** | Verifies the custom interface generation pipeline. | **Pass** if `rosidl_generate_interfaces` is present and `catkin` macros are removed. |
| **test_package_xml_format** | Enforces ROS 2 engineering standards and interface visibility. | **Pass** if `format="3"` is used and `rosidl_interface_packages` is declared. |
| **test_no_rospy_anywhere** | Ensures a clean migration without "zombie" ROS 1 code. | **Pass** if no strings of `rospy` or `catkin` exist in the migrated files. |

