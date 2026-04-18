# Benchmark Task 009: RoboFleet msg2fbs Cross-Language Consistency

## 1. Brief Description
This task involves migrating the core components of the `robofleet` system from **ROS 1** to **ROS 2 (Humble/Iron)**. The primary target is the `msg2fbs` tool, which dynamically converts ROS Message (.msg) definitions into FlatBuffers Schemas (.fbs).

This is a **System Level** challenge. It tests the LLM's ability to maintain strict synchronization between **procedural logic** (the Python generator) and **static interface contracts** (the FBS schema). A failure to align field names, memory layouts (struct vs. table), or namespaces across these files will result in a complete breakdown of the system's serialization pipeline.

---
source code file:
```https://github.com/ut-amrl/robofleet/blob/master```
---


## 2. Hollowing Strategy & Interdependence

To evaluate architectural integrity over simple code completion, we have removed entire functional blocks to break local context:

### **Hole A: Core Transport Generation (`msg2fbs.py` -> `gen_support`)**
* **Scope**: The entire body of the `gen_support()` function.
* **Reasoning**: Forces the model to define base types from scratch. In ROS 2, time primitives shifted from `secs/nsecs` to `sec/nanosec`.
* **Coupling**: The model must choose between `struct` and `table`. If the chosen structure or naming differs from the static `schema.fbs`, the system integration test will fail.

### **Hole B: Namespace Parsing (`msg2fbs.py` -> `Type.__init__`)**
* **Scope**: The `Type` class constructor.
* **Reasoning**: ROS 2 introduces a nested `msg` directory in package paths (e.g., `std_msgs/msg/Header`). 
* **Coupling**: This tests if the model implements a generic logic to convert ROS slashes `/` to FlatBuffers dots `.` across file boundaries, rather than relying on ROS 1 hardcoded patterns.

### **Hole C: Primitive Schema Definitions (`schema.fbs` -> `RosTime/RosDuration`)**
* **Scope**: Definitions for `RosTime` and `RosDuration`.
* **Reasoning**: Acts as the "Anchor" for the static schema. It must perfectly mirror the Python generator's output.

### **Hole D: ROS 2 Header Adaptation (`schema.fbs` -> `table Header`)**
* **Scope**: The full `Header` table definition.
* **Reasoning**: Tests knowledge of the ROS 2 `std_msgs/Header` evolution (removal of `seq`) and ensures it correctly references the updated `RosTime` struct.

---

## 3. Oracle Testcase Design & Expected Outcomes

The Oracle uses strict regex-based static analysis to verify synchronization and ROS 2 standards.

| Testcase | Design Logic | Expected Outcome (Pass Criteria) |
| :--- | :--- | :--- |
| **test_time_struct_field_sync** | **Cross-File Sync**: Matches the yielded strings in Python against the FBS definitions. | Both files use `sec` and `nanosec`; FBS must use `struct` for binary alignment. |
| **test_header_schema_no_seq** | **Evolution Check**: Verifies the removal of legacy ROS 1 fields. | The `Header` table in `schema.fbs` contains NO `seq` field and correctly uses `stamp:RosTime`. |
| **test_namespace_conversion_logic** | **Logic Genericity**: Ensures the model didn't use "lazy" hardcoding. | Source code must contain `.replace("/", ".")` or equivalent path-handling logic. |
| **test_no_ros1_naming_leakage** | **Hallucination Guard**: Scans for legacy ROS 1 terminology. | Zero occurrences of the standalone words `secs` or `nsecs` in the entire codebase. |
| **test_fbs_required_attribute** | **Semantic Integrity** | Non-scalar fields (like `stamp`) must include the `(required)` attribute in the FBS. |

