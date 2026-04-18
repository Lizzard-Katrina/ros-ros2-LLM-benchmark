# Task 004: Integrated Navigation & Parameter System (Turtlesim)

## 1. Brief Description
This task evaluates a model's **System-Level** integration capabilities within ROS 2 Humble. Unlike simple code completion, this task requires the model to maintain a "Sense-Plan-Act" pipeline across four different files. The model must bridge C++ logic with build configurations (`CMakeLists.txt`) and dependency manifests (`package.xml`). The core objective is to implement a closed-loop square-drawing controller while ensuring the GUI reacts to dynamic ROS 2 parameter events.

---
source code file(base):
```http://github.com/ros/ros_tutorials/blob/rolling/turtlesim```

## 2. Hole Design & Rationale

### **Hole 1: Perception & Parameters (`turtle_frame.cpp`)**
* **Target**: The bridge between ROS 2 parameters and Qt6 GUI updates.
* **Rationale**: Tests if the model understands `rcl_interfaces` and the event-driven nature of ROS 2. It must declare parameters with `ParameterDescriptor` constraints and implement a callback that triggers a GUI `update()` specifically for this node's events.

### **Hole 2: Navigation & Control Logic (`draw_square.cpp`)**
* **Target**: A Finite State Machine (FSM) for closed-loop motion.
* **Rationale**: This is the "brain." The model must transition between `FORWARD` and `TURN` states by comparing `current_pose_` feedback with a calculated `goal_pose_`. It tests geometric precision (PI/2 turns) and the ability to handle asynchronous service results (the `reset` call).

### **Hole 3: System Build Configuration (`CMakeLists.txt`)**
* **Target**: The engineering glue.
* **Rationale**: A frequent failure point for LLMs. It must correctly link Qt6 components with `AUTOMOC` and, most importantly, use `rosidl_target_interfaces` to link custom message headers to the executables—a requirement often missed by those only familiar with ROS 1.

### **Hole 4: Dependency Manifest (`package.xml`)**
* **Target**: The metadata layer.
* **Rationale**: Tests "back-propagation" of requirements. If the model uses `geometry_msgs` or `turtlesim_msgs` in C++, it must manually declare them here. Failure breaks the ROS 2 workspace's dependency resolution.

---

## 3. Test Case Design & Expected Outcomes

The Oracle uses pattern matching to validate semantic concepts without execution:

| Test Case | Design Logic | Expected Outcome |
| :--- | :--- | :--- |
| **`test_no_ros1_remnants`** | Scans for legacy keywords like `ros::NodeHandle` or `catkin`. | **Strict Cleanliness**: No ROS 1 API leakage in a Humble task. |
| **`test_parameter_logic`** | Checks for `ParameterDescriptor` and `parameter_events`. | **Modern API**: Proper use of descriptors for constraints. |
| **`test_navigation_geometry`** | Validates the mathematical essence of the square pattern. | **Geometric Logic**: Use of `1.57` or `PI/2` and `current_pose_` feedback. |
| **`test_cmake_integration`** | Verifies ROS 2 specific build requirements. | **Interface Linking**: Explicit use of `rosidl_target_interfaces`. |
| **`test_package_xml_deps`** | Cross-references C++ usage with XML declarations. | **Full Dependency Graph**: Presence of `geometry_msgs`, `turtlesim_msgs`, and `qt6`. |
