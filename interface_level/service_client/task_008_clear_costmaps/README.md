# ROS1 → ROS2 Translation Task: Clear Costmap Behavior (Service Client)

## 1. Brief Description

This task evaluates whether an LLM can correctly translate a **ROS1 behavior-level costmap clearing logic** into a semantically equivalent **ROS2 service-client-based implementation**.

The original ROS1 code uses:

- private `NodeHandle` parameters  
- dynamic construction of `layer_names`  
- boolean-controlled selection of layers (`obstacles`, `static_map`)  
- triggering a clear-costmap behavior  

A shallow translation that only converts syntax (e.g., replacing headers or using a generic ROS2 client) is insufficient. The translated code must preserve the **behavioral semantics** of:

- which layers are cleared  
- under what conditions  
- how the clearing is triggered  

This task focuses on **interface-level and behavior-level equivalence**, not just API migration.

---
source code: 
`https://github.com/ros-planning/navigation/blob/noetic-devel/clear_costmap_recovery/test/clear_tester.cpp`

## 2. Why We Hollow Out the Source Code

We intentionally hollow out (`testClearBehavior`) because this function is the **core semantic unit** of the original logic.

In ROS1, this function:

- sets parameters (`reset_distance`, `layer_names`)  
- builds a list of layers to clear  
- selects layers conditionally  
- triggers the clear behavior  

If we left this fully implemented, an LLM could copy or pattern-match without reasoning.

### Goal of Hollowing

The goal is to force the model to:

- reconstruct how ROS1 parameter-based behavior maps to ROS2 patterns  
- decide how to encode layer selection in ROS2 (parameters or request fields)  
- correctly trigger a clear-costmap action via ROS2 services  
- preserve conditional logic tied to input flags  

In other words, this tests **semantic translation**, not token translation.

---

## 3. Oracle Testcases

We use 6 regex-based oracle tests.  
They are **static**, fast (<1s), and focus on semantic signals rather than exact syntax.

Each test captures one critical concept from the original ROS1 behavior.

---

# Testcase 1  
## No ROS1 Core API Remnants

### Design Rationale

A valid ROS2 translation must not rely on ROS1 runtime APIs.  
Keeping ROS1 calls indicates superficial or incorrect migration.

We explicitly forbid:

- `ros::NodeHandle`  
- `ros::init`  
- `ros::spin`  
- `ROS_INFO/WARN/...` macros  
- `<ros/ros.h>`  

### Expected Outcome

A correct translation:

- includes `rclcpp`  
- uses ROS2 node and lifecycle APIs  
- contains no ROS1 runtime calls  

---

# Testcase 2  
## `testCountLethal` Calls `testClearBehavior`

### Design Rationale

We want to ensure the original control flow structure is preserved.

In ROS1, `testCountLethal` is the driver that invokes the clearing behavior.  
If the translation bypasses this structure, the semantic mapping is likely broken.

### Expected Outcome

A correct translation:

- still uses `testClearBehavior` as the clearing entry point  
- preserves the test-driven flow  

---

# Testcase 3  
## `testClearBehavior` Exists and Is Not a Stub

### Design Rationale

LLMs often leave hollowed functions empty or minimally filled.

We require that:

- the function exists  
- it performs real ROS2 work  
- it is not a placeholder  

### Expected Outcome

A correct translation:

- creates parameters or a service client  
- prepares request data or configuration  
- triggers actual clearing logic  

---

# Testcase 4  
## `reset_distance` Semantics Preserved

### Design Rationale

`reset_distance` is a core parameter controlling clearing radius.

Losing this changes behavior significantly.

We allow two valid ROS2 mappings:

- ROS2 parameter `"reset_distance"`  
- request field assignment (e.g., `req->distance = ...`)  

### Expected Outcome

A correct translation:

- preserves the notion of a clearing distance  
- wires the function argument into the clearing configuration  

---

# Testcase 5  
## Layer Name Semantics (`obstacles` / `static_map`)

### Design Rationale

This is the most behavior-specific part.

ROS1 logic dynamically builds `layer_names` based on:

```cpp
if (obstacles) ...
if (static_map) ...
```
### Expected Outcome
A correct translation:
- includes "obstacles" and/or "static_map"

- selects layers conditionally

- does not hardcode a single layer

- preserves boolean-controlled layer selection
# Testcase 6
##Service Call + Proper Waiting

### Design Rationale

Triggering the clear behavior is essential.

A translation that configures parameters but never calls a service is incomplete.

### Expected Outcome

A correct translation:

- actually triggers clearing

- does not fire-and-forget without waiting

- uses standard ROS2 client patterns
