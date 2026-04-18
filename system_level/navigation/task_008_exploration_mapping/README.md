# Task 008: Differential Drive Hardware Interface Migration (ROS 1 to ROS 2 Control)

## 1. Brief Description
This task involves migrating a mobile robot's hardware interface from a standalone ROS 1 Node to a ROS 2 `hardware_interface::SystemInterface` plugin. The challenge is not just translating API calls, but re-architecting the **lifecycle management** and **resource export mechanism**. 

The model must synchronize the C++ hardware plugin with a YAML controller configuration to ensure the `controller_manager` can successfully load, command, and read from the I2C-based motor drivers.

---
source code file:
1. ```https://github.com/bandasaikrishna/Autonomous_Mobile_Robot/blob/main/mobile_robot_autonomous_navigation/config/controllers.yaml```
2. ```https://github.com/bandasaikrishna/Autonomous_Mobile_Robot/blob/main/mobile_robot_autonomous_navigation/src/robot_hardware_interface_node.cpp```


## 2. Hollowing Design & Philosophy
The task intentionally preserves the low-level I2C business logic (e.g., `readBytes`, `writeData`) while hollowing out the middleware integration points. This forces the model to perform "System Integration" rather than "Driver Writing."

### A. Hole 1: Plugin Architecture (Class Header)
* **Hollowed**: The class inheritance and member declarations.
* **Purpose**: To verify if the model understands that ROS 2 hardware interfaces must inherit from `SystemInterface` and provide specific overrides (`on_init`, `export_state_interfaces`, etc.) instead of being a standard `rclcpp::Node`.

### B. Hole 2: Lifecycle & Resource Mapping (Source Logic)
* **Hollowed**: The constructor, initialization logic, and interface exporting.
* **Purpose**: In ROS 2, the system does not use `registerInterface()`. The model must correctly map internal buffers (position/velocity) to `StateInterface` and `CommandInterface` objects. This tests the "contract" between hardware and controllers.

### C. Hole 3: Execution Model Flip (Main Function)
* **Hollowed**: The entire `main()` function.
* **Purpose**: A critical "trap." In ROS 2 Control, hardware interfaces are **dynamic libraries (.so)** loaded by the `controller_manager`, not standalone executables. The model must remove the `main()` function and use `PLUGINLIB_EXPORT_CLASS`.

### D. Hole 4: Controller Orchestration (YAML Config)
* **Hollowed**: The entire `controllers.yaml`.
* **Purpose**: To enforce **Cross-File Synchronization**. The names of the joints (e.g., `left_wheel_joint`) and the types of interfaces (e.g., `velocity`) defined here must perfectly match the strings used in the C++ code.

---

## 3. Oracle Testcases & Expected Outcomes

The validation uses semantic pattern matching to ensure the architectural integrity of the migration.

| Test Case | Strategy / Concept | Expected Outcome (Success Criteria) |
| :--- | :--- | :--- |
| `test_architecture_transformation` | **Plugin vs Node** | Presence of `SystemInterface` inheritance and **absence** of a `main()` function. |
| `test_lifecycle_methods` | **State Machine** | Presence of `on_init`, `export_state_interfaces`, and `export_command_interfaces` overrides. |
| `test_interface_matching` | **Data Contract** | Correct usage of `HW_IF_POSITION` and `HW_IF_VELOCITY` constants in C++. |
| `test_ros2_read_write_signatures`| **API Accuracy** | `read()` and `write()` must accept `(const rclcpp::Time&, const rclcpp::Duration&)` to override the base class. |
| `test_yaml_structure_and_sync` | **System Alignment** | YAML must use the `ros__parameters` nesting and define a `DiffDriveController` targeting the correct joint names. |
| `test_legacy_cleanup` | **Anti-Leakage** | Total absence of `ros::NodeHandle` or `registerInterface` to ensure no "franken-code" exists. |
# Task 008: Differential Drive Hardware Interface Migration (ROS 1 to ROS 2 Control)

## 1. Brief Description
This task involves migrating a mobile robot's hardware interface from a standalone ROS 1 Node to a ROS 2 `hardware_interface::SystemInterface` plugin. The challenge is not just translating API calls, but re-architecting the **lifecycle management** and **resource export mechanism**. 

The model must synchronize the C++ hardware plugin with a YAML controller configuration to ensure the `controller_manager` can successfully load, command, and read from the I2C-based motor drivers.

---

## 2. Hollowing Design & Philosophy
The task intentionally preserves the low-level I2C business logic (e.g., `readBytes`, `writeData`) while hollowing out the middleware integration points. This forces the model to perform "System Integration" rather than "Driver Writing."

### A. Hole 1: Plugin Architecture (Class Header)
* **Hollowed**: The class inheritance and member declarations.
* **Purpose**: To verify if the model understands that ROS 2 hardware interfaces must inherit from `SystemInterface` and provide specific overrides (`on_init`, `export_state_interfaces`, etc.) instead of being a standard `rclcpp::Node`.

### B. Hole 2: Lifecycle & Resource Mapping (Source Logic)
* **Hollowed**: The constructor, initialization logic, and interface exporting.
* **Purpose**: In ROS 2, the system does not use `registerInterface()`. The model must correctly map internal buffers (position/velocity) to `StateInterface` and `CommandInterface` objects. This tests the "contract" between hardware and controllers.

### C. Hole 3: Execution Model Flip (Main Function)
* **Hollowed**: The entire `main()` function.
* **Purpose**: A critical "trap." In ROS 2 Control, hardware interfaces are **dynamic libraries (.so)** loaded by the `controller_manager`, not standalone executables. The model must remove the `main()` function and use `PLUGINLIB_EXPORT_CLASS`.

### D. Hole 4: Controller Orchestration (YAML Config)
* **Hollowed**: The entire `controllers.yaml`.
* **Purpose**: To enforce **Cross-File Synchronization**. The names of the joints (e.g., `left_wheel_joint`) and the types of interfaces (e.g., `velocity`) defined here must perfectly match the strings used in the C++ code.

---

## 3. Oracle Testcases & Expected Outcomes

The validation uses semantic pattern matching to ensure the architectural integrity of the migration.

| Test Case | Strategy / Concept | Expected Outcome (Success Criteria) |
| :--- | :--- | :--- |
| `test_architecture_transformation` | **Plugin vs Node** | Presence of `SystemInterface` inheritance and **absence** of a `main()` function. |
| `test_lifecycle_methods` | **State Machine** | Presence of `on_init`, `export_state_interfaces`, and `export_command_interfaces` overrides. |
| `test_interface_matching` | **Data Contract** | Correct usage of `HW_IF_POSITION` and `HW_IF_VELOCITY` constants in C++. |
| `test_ros2_read_write_signatures`| **API Accuracy** | `read()` and `write()` must accept `(const rclcpp::Time&, const rclcpp::Duration&)` to override the base class. |
| `test_yaml_structure_and_sync` | **System Alignment** | YAML must use the `ros__parameters` nesting and define a `DiffDriveController` targeting the correct joint names. |
| `test_legacy_cleanup` | **Anti-Leakage** | Total absence of `ros::NodeHandle` or `registerInterface` to ensure no "franken-code" exists. |

