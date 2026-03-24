# Task 007: AMCL Parameter Server & Dynamic Reconfigure Migration

## 1. Brief Description
This task involves migrating the Adaptive Monte Carlo Localization (AMCL) node's parameter management from the ROS 1 "Split Model" (Static `ros::param` + Dynamic `dynamic_reconfigure`) to the ROS 2 **Unified Parameter System**. In the Noetic version of AMCL, parameters are scattered between the constructor and a dedicated `reconfigureCB` callback. The goal is to consolidate these into the ROS 2 `declare_parameter` API and the `add_on_set_parameters_callback` mechanism, ensuring the node remains configurable at runtime while maintaining strict safety constraints.
---
source code:
```https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp#L349```

## 2. Excavation Strategy & Purpose

### Excavation Strategy
We have "excavated" (removed) two critical sections of the `amcl_node.cpp` file:
* **The Initialization Block:** The logic in the constructor that originally used `private_nh_.param()` to fetch static parameters (Noetic L471-L600).
* **The Reconfigure Callback:** The entire `reconfigureCB` function and its server binding (`dynamic_reconfigure::Server`), which handled runtime updates (Noetic L624-L768).

### Purpose of Testing the Parameter Server
Testing the Parameter Server migration serves three main objectives:
1.  **Consolidated API Awareness:** To verify if the model understands that ROS 2 eliminates the distinction between "static" and "dynamic" parameters—everything must be declared.
2.  **Middleware-State Synchronization:** To ensure that internal member variables (like `min_particles_`) are correctly synchronized with the middleware state via a dedicated callback.
3.  **Logical Integrity (Perception-Server Logic):** To test if the model retains **Business Logic Constraints** (e.g., `min_particles` must not exceed `max_particles`) during the migration, preventing the node from entering an unstable state.

## 3. Oracle Test Design & Expected Outcomes

Each test case uses regex-based pattern matching to validate semantic concepts without the overhead of compilation.

| Test Case | Design Rationale (Semantic Concept) | Expected Outcome (Pass Criteria) |
| :--- | :--- | :--- |
| **test_param_declaration** | **Explicit Declaration**: ROS 2 requires parameters to be declared before they are accessible. | Matches `this->declare_parameter<int>("min_particles", ...)` or similar for all core AMCL params. |
| **test_callback_registration** | **Lifecycle Binding**: Parameters must be linked to a handler to allow runtime tuning. | Code must contain `add_on_set_parameters_callback` using `std::bind` or a lambda. |
| **test_min_max_logic** | **Constraint Enforcement**: Validates that the model preserves the specific AMCL rule: `min <= max`. | A regex match for a comparison between `min_particles` and `max_particles` with a logic branch for `.successful = false`. |
| **test_type_safety** | **Variant Handling**: ROS 2 parameters are stored in variants. This checks the use of correct getters. | Presence of `.as_int()`, `.as_double()`, or `.as_string()` based on the parameter type. |
| **test_no_legacy_artifacts** | **Zero-Legacy Migration**: Ensures the model doesn't "hallucinate" ROS 1 structures. | **Failure** if any instance of `private_nh_`, `dynamic_reconfigure`, or `AMCLConfig` is found. |
| **test_feedback_reason** | **Human-Readable Error**: ROS 2 allows providing reasons for rejecting parameter changes. | The `SetParametersResult` must include an assignment to the `.reason` field during validation failure. |

---
