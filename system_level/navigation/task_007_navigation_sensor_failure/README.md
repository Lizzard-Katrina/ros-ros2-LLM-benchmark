# Task 007: Turtlesim System-Deep Sync (ROS 1 to ROS 2 Migration)

## 1. Brief Description
This task involves migrating the core GUI logic of `turtlesim` (specifically the `TurtleFrame` class) from ROS 1 to ROS 2 Humble. Unlike simple topic translation tasks, this is a **System-Level Synchronization** challenge. 

The goal is to re-architect how the simulation interacts with the ROS 2 middleware. The model must bridge the gap between the **Qt event loop** and the **ROS 2 execution model**, requiring the correct implementation of a `SingleThreadedExecutor`, explicit parameter declarations with safety constraints, and synchronized service callbacks across header and source files.

---
source code file:
1. ```https://github.com/ros/ros_tutorials/blob/rolling/turtlesim/src/turtle_frame.cpp```
2. ```https://github.com/ros/ros_tutorials/blob/rolling/turtlesim/include/turtlesim/turtle_frame.hpp```


## 2. Hollowing Design Philosophy
The task "hollows out" three critical architectural intersections where ROS 1 and ROS 2 differ fundamentally:

### A. Infrastructure Skeleton (Header `.hpp`)
* **What's missing**: Private members for `rclcpp::Node`, `rclcpp::executors::SingleThreadedExecutor`, and all Service/Subscription pointers.
* **Design Intent**: To test if the model understands that a ROS 2 class-based component requires an internal execution mechanism (`Executor`) and a Node handle to manage its own callbacks, rather than relying on a global background `ros::spin()`.

### B. System Linkage & Parameter Safety (Source `.cpp` - Constructor)
* **What's missing**: The entire initialization block for Node-to-Executor linkage and Parameter declaration.
* **Design Intent**: 
    1.  **Node-Executor Association**: In ROS 2, services won't trigger unless the Node is added to an Executor. 
    2.  **Explicit Declaration**: ROS 2 requires parameters to be declared before use. We force the use of `rcl_interfaces::msg::IntegerRange` to test the model's knowledge of ROS 2 best practices for parameter validation.

### C. Non-Blocking Runtime Loop (Source `.cpp` - `onUpdate`)
* **What's missing**: The internal logic of the timer-driven update loop.
* **Design Intent**: This is a "trap" for models that only understand basic API translation. If the model uses a blocking `rclcpp::spin()`, the Qt GUI will freeze. The model must use `executor_.spin_some()` to maintain UI responsiveness.

---

## 3. Test Cases & Expected Outcomes

The validation relies on **Oracle Pattern Matching** with anchor constraints. Below are the test concepts and what they expect to find:

| Test Case | Strategy / Concept | Expected Outcome (Success Criteria) |
| :--- | :--- | :--- |
| `test_executor_declaration` | Header Membership | Presence of `rclcpp::executors::SingleThreadedExecutor executor_;` in the private section. |
| `test_callback_signatures` | Interface Consistency | Method signatures in `.hpp` must use `(Request::SharedPtr, Response::SharedPtr)`. |
| `test_executor_node_linkage`| System Plumbing | Source code must contain `executor_.add_node(nh_)` (or pointer equivalent). |
| `test_parameter_safety` | ROS 2 Best Practice | Usage of `nh_->declare_parameter` paired with `IntegerRange` logic (0-255). |
| `test_non_blocking_spin` | Execution Logic | The `onUpdate` loop must call `executor_.spin_some()` to ensure the GUI remains active. |
| `test_service_binding` | Standardized Style | Services must be bound using `std::bind` with `std::placeholders::_1` and `_2` as per constraints. |
| `test_anti_leakage` | Legacy Cleanup | Total absence of ROS 1 tokens like `ros::NodeHandle` or `ros::ok()`. |
| `test_namespace_sync` | Type Accuracy | Correct use of nested namespaces, specifically `::srv::` for all service types. |

