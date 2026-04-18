# Task 005: NavFn Global Planner Plugin Migration (ROS 2)

## 1. Brief Description
This task involves migrating the `navfn` global planner plugin from ROS 1 to **ROS 2 Humble**. The goal is to establish a robust, thread-safe global planning interface that integrates with the `pluginlib` architecture. You are required to maintain semantic consistency across the interface definition (`nav_core`), the plugin implementation (`navfn`), and the `ament_cmake` build configuration.

---
source file:
```https://github.com/ros-planning/navigation/blob/noetic-devel```

## 2. Hole Design & Semantic Logic

### Hole 1: Interface Definition (`nav_core/base_global_planner.h`)
* **Concept**: Define a pure virtual contract for all global planners.
* **Logic**:
    * Implement **pure virtual** (`= 0`) methods for `initialize` and `makePlan`.
    * Transition to **ROS 2 namespaces**: Use `geometry_msgs::msg::PoseStamped` instead of legacy types.
    * Ensure **Virtual Destructor**: Prevent memory leaks and ensure proper cleanup when the plugin is unloaded by the navigation stack.

### Hole 2: Plugin Implementation (`navfn/src/navfn_ros.cpp`)
* **Concept**: Implement the core planning algorithm logic with ROS 2 primitives.
* **Logic**:
    * **Thread Safety**: Must use `std::lock_guard` with the class `mutex_` to prevent race conditions during costmap updates.
    * **Frame Validation**: Ensure that input `start` and `goal` poses match the `global_frame_` before processing.
    * **Coordinate Transform**: Map world coordinates (double) to costmap indices (unsigned int) using `worldToMap`.
    * **Memory Management**: Set goals using stack-allocated arrays or `std::array` instead of raw `new` calls to prevent leaks.
    * **Path Extraction**: Populate the `plan` vector by iterating through the Dijkstra potential field and converting indices back to world poses.

### Hole 3: Build System (`navfn/CMakeLists.txt`)
* **Concept**: Configure `ament_cmake` for library building and plugin export.
* **Logic**:
    * **Ament Migration**: Replace all `catkin_` macros with `ament_` equivalents.
    * **Target Dependencies**: Correct linking of `rclcpp`, `nav_core`, and `pluginlib`.
    * **Plugin Registration**: Install `bgp_plugin.xml` to the `share` directory to allow the ROS 2 environment to discover the planner class.

---

## 3. Test Case Design & Expected Outcomes

The Oracle Test validates code through pattern matching to ensure conceptual correctness:

| Test Case | Design Logic | Expected Outcome |
| :--- | :--- | :--- |
| `test_interface_signature` | Validates the pure virtual contract and ROS 2 types. | Matches `virtual bool makePlan(...) = 0;` and `msg::PoseStamped`. |
| `test_thread_safety` | Ensures the implementation is safe for multi-threaded nav stacks. | Matches `std::lock_guard` or `std::unique_lock` wrapping `mutex_`. |
| `test_build_system_bridge` | Detects "dirty" migrations from ROS 1 codebases. | **Fails** if `catkin_` strings are present; **Passes** if `ament_package()` is found. |
| `test_no_raw_new_leak` | Enforces modern C++ memory safety standards. | **Fails** if `new int[` is used; expects stack-based coordinate management. |
| `test_plugin_installation` | Checks the runtime discoverability of the plugin. | Matches `install(FILES bgp_plugin.xml DESTINATION share/${PROJECT_NAME})`. |
| `test_semantic_flow` | Verifies the operational order of the planning logic. | Matches `worldToMap` occurring before `planner_->setGoal`. |

