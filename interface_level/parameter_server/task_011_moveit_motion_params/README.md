# Task 011: MoveItCpp High-Level Environment Migration

## 1. Brief Description
Task 011 is an advanced architectural benchmark focused on migrating the initialization backbone of a MoveIt application from ROS 1 to ROS 2. Unlike simple API mapping, this task requires the model to reconstruct the entire execution environment for `MoveItCpp`. This includes configuring a "locked" ROS 2 Node to accept dynamic parameters, managing asynchronous execution to prevent deadlocks, and correctly linking core MoveIt 2 components within their new namespacing and dependency injection patterns.
---
source code:
```https://github.com/moveit/moveit_tutorials/blob/master/doc/moveit_cpp/src/moveit_cpp_tutorial.cpp```
## 2. Excavation Strategy (Logic-Loop Reconstruction)
The excavation removes the entire initialization sequence from `main()`—roughly 40 lines of code—leaving only a high-level goal. This forces the model to synthesize multiple ROS 2 concepts into a single functional block:

* **From Micro-Management to Macro-Architecture**: We removed granular TODOs (e.g., "Step 1: Create NodeOptions"). The model must autonomously recognize that `MoveItCpp` cannot function without specific `NodeOptions` and a background `Executor`.
* **The Initialization Dependency Chain**: The excavation covers the dependency flow: 
    `rclcpp::init` -> `NodeOptions` -> `Node` -> `Executor/Thread` -> `MoveItCpp` -> `PlanningComponent`.
* **Infrastructure "Black-Box"**: By excavating the setup and leaving the downstream visualization code intact, we test if the model can provide the exact object pointers (`moveit_cpp_ptr`, `node`) and states required by the rest of the tutorial.

## 3. Oracle Test Design and Expected Outcomes

The Oracle suite uses a pre-processor to strip comments, ensuring only functional code is evaluated.

| Testcase | Design Principle | Expected Outcome / Passing Criteria |
| :--- | :--- | :--- |
| **Node Lifecycle Setup** | Validates the "unlocked" parameter state. | Presence of `automatically_declare_parameters_from_overrides(true)` and `allow_undeclared_parameters(true)` passed into the Node constructor. |
| **Namespace Accuracy** | Detects "ROS 1 Hallucinations" regarding library paths. | Strict absence of `moveit::planning_interface::MoveItCpp`. Must use the correct ROS 2 `moveit_cpp::MoveItCpp` namespace. |
| **Async Execution** | Ensures the program doesn't deadlock on startup. | Implementation of a background execution mechanism, such as a `std::thread` running `rclcpp::spin(node)` or a `MultiThreadedExecutor`. |
| **Scene Availability** | Validates full system integration. | Explicit call to `providePlanningSceneService()`. This ensures the planning environment is discoverable by external tools like RViz. |
| **Clean Migration** | Checks for "Frankenstein code" (mixing ROS 1 & 2). | Absolute absence of legacy symbols like `ros::NodeHandle`, `ros::init`, or `AsyncSpinner` in the executable code. |
| **Message Namespacing** | Checks for proper ROS 2 IDL paths. | Correct usage of the nested `::msg::` namespace for all ROS messages (e.g., `geometry_msgs::msg::PoseStamped`). |

## 4. How to Pass
To pass this task, the model must go beyond syntax translation. It must demonstrate "Domain Awareness" of MoveIt 2's specific requirements:
1.  **Unlock the Parameters**: Without `NodeOptions` overrides, the YAML configs will fail to load.
2.  **Thread the Spinner**: MoveItCpp's constructor waits for internal topics; without a background spin, the program hangs forever at 0%.
3.  **Correct the Path**: Use `moveit_cpp::` instead of the legacy `planning_interface::`.
