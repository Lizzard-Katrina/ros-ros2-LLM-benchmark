# Task 013: SLAM GMapping - Professional ROS 2 Parameter Infrastructure

## 1. Brief Description
Task 013 is the "Capstone" project for the ROS 2 Parameter Server track. It involves migrating the configuration backbone of the OpenSlam GMapping wrapper. Unlike previous tasks, GMapping contains over 20 interconnected parameters (Laser Physics, Motion Noise, and Grid Map Resolution). The goal is to move from the legacy ROS 1 `NodeHandle` approach to a centralized, validated, and type-safe ROS 2 `rclcpp::Node` parameter system.
---
source code file:
```https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/src/slam_gmapping.cpp```
## 2. Excavation Strategy: Architecture Over Translation
The excavation removes the entire `init()` sequence where parameters are loaded. This is a **High-Density Logic Gap** designed to test:

* **Registry Pattern**: Can the model correctly implement the **Declare-then-Get** lifecycle? In ROS 2, attempting to get a parameter without declaring it first results in a runtime exception.
* **API Purity**: The excavation forces the model to realize that `private_nh_` (a ROS 1 artifact) no longer exists. The model must use `this->` or the node's native methods.
* **Domain-Specific Logic**: Beyond just "loading strings," the model must recognize that SLAM requires **Physical Consistency**. For example, the usable range of a laser (`maxUrange`) cannot logically exceed its physical hardware limit (`maxRange`).

## 3. Oracle Test Design & Expected Outcomes

The Oracle suite uses advanced regex to detect structural integrity rather than just keyword presence.

| Test Case | Strategy / Intent | Expected Outcome (To Pass) |
| :--- | :--- | :--- |
| **ROS 2 Lifecycle** | Verifies the sequence of parameter registration. | Must use `declare_parameter` before `get_parameter`. |
| **No Fake NodeHandle** | Detects "Hybrid" code (hallucinating ROS 1 objects). | The code **must not** contain the variable `private_nh_`. |
| **Physical Validation** | Checks for "Engineering Intuition" regarding sensors. | Must contain an `if` block comparing `maxUrange` and `maxRange` with a corrective assignment. |
| **Explicit Type Casting** | Enforces strict C++ type safety. | Must use explicit getters like `.as_double()` or `.as_int()` instead of ambiguous overloads. |
| **Logging Modernization** | Validates the transition to the ROS 2 Logger API. | Presence of `RCLCPP_` macros or `get_logger()`; **Zero** instances of legacy `ROS_WARN`. |
| **Value Consistency** | Ensures SLAM-specific defaults are preserved. | `temporalUpdate` should be initialized (often to `-1.0` to disable) and `delta` must be `0.05`. |
| **Member Mapping** | Checks if data actually enters the SLAM engine. | Parameters must be assigned to class members (e.g., `this->particles_ = ...`). |

## 4. Expected Final Result
A successful implementation will show a clean block of code where all 20+ parameters are declared with default values, retrieved using explicit type-casting, and validated for physical sanity before being assigned to the class member variables. This ensures the SLAM node is robust, configurable via YAML, and safe for real-world robotic deployment.
