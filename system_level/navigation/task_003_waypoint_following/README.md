# Task 003: Actionlib to rclcpp_action Migration

## 1. Brief Description
This task focuses on the migration of a ROS 1 Action Server to ROS 2 Humble. The original node `add_two_ints_server.cpp` uses the `actionlib` library to provide a preemptible service that sums two integers. In ROS 2, this requires a significant structural shift from simple callbacks to a managed asynchronous state machine using `rclcpp_action`.
---
source code:
```https://github.com/ros/actionlib/tree/noetic-devel```

## 2. "Hole" Design Strategy
We utilize coarse-grained logical block removal to test the model's ability to maintain system-wide consistency across multiple files:

* **package.xml (Dependency Hole)**: 
    * **Range**: Lines 22-38.
    * **Logic**: Removes all ROS 1 buildtool and dependency tags. The model must provide the ROS 2 `ament_cmake` buildtool and the correct `rclcpp_action`, `action_msgs`, and `rosidl` dependencies.
* **CMakeLists.txt (Build System Hole)**:
    * **Range**: Line 4 to EOF.
    * **Logic**: Removes the entire ROS 1 `catkin` configuration. The model must reconstruct the `rosidl_generate_interfaces` pipeline and link the generated C++ headers to the server executable.
* **add_two_ints_server.cpp (Logic Hole)**:
    * **Range**: Lines 37-58.
    * **Logic**: Removes headers and the ROS 1 `ServiceServer` initialization. The model must implement the ROS 2 Action Server lifecycle, specifically handling `GoalResponse`, `CancelResponse`, and the `execute` logic using `ServerGoalHandle`.

## 3. Oracle Test Design & Expected Outcomes
The Oracle script (`test_oracle_ros2.py`) validates the migration through four key checks:

| Test Case | Design Rationale | Expected Outcome to Pass |
| :--- | :--- | :--- |
| `test_dependency_sync` | Ensures manifest and build scripts are aligned. | `rclcpp_action` and `action_msgs` must exist in both `package.xml` and `CMakeLists.txt`. |
| `test_interface_sync` | Verifies the IDL-to-C++ header bridge. | `CMakeLists.txt` must call `rosidl_generate_interfaces` for the `.action` files used in C++. |
| `test_cpp_action_logic` | Validates the ROS 2 asynchronous pattern. | C++ code must use `ServerGoalHandle`, `ACCEPT_AND_EXECUTE`, and call `.succeed()` with the correct sum. |
| `test_build_migration` | Confirms total removal of legacy middleware. | No `catkin` keywords should remain; `ament_package()` must be the final call in CMake. |
