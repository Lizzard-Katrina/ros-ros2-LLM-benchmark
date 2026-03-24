# Benchmark Task 002: ScenarioStateBase Migration (Base Class Architecture)

## 1. Brief Description
This task involves migrating a base class, `ScenarioStateBase`, from ROS 1 to ROS 2. This class serves as the foundation for various scenario states in an execution manager. Unlike a leaf-node state, a base class migration is critical because it defines how all derived states will handle ROS 2 Node handles, logging, and shared communication interfaces (Knowledge Base and Ontology).

The challenge lies in transitioning from global `rospy` singletons to a **Dependency Injection** model where the `Node` handle must be managed correctly.
---
source code file
```https://github.com/b-it-bots/mas_execution_manager/blob/master/ros/src/mas_execution_manager/scenario_state_base.py```

## 2. Hollowing Strategy (Hole-filling)
The hollowing targets the core initialization and lifecycle logic to evaluate if the LLM understands ROS 2 resource management.

- **Scope:** The entire body of the `__init__` method (excluding the call to the parent constructor) and the timestamp logic in `save_current_state`.
- **Injected TODO:** A coarse-grained prompt that defines **Functional Goals** (migration of Pub/Sub/Params) and **Style Constraints** (Variable naming and absence of type hints).

### Why this approach?
By removing the internal setup, we force the LLM to:
1.  Correctly implement the ROS 2 **Parameter Declaration** (which didn't exist in ROS 1).
2.  Map ROS 1 **Latching** behavior to ROS 2 **QoS Durability** (Transient Local).
3.  Ensure the data flow to external interfaces (`DomesticKBInterface`) remains consistent with the original logic.

## 3. Oracle Test Design & Expected Outcomes

The Oracle tests are designed to be **Strict** and **Logic-Focused**, catching common LLM "hallucinations" or "shortcuts" during migration.

| Test Case | Design Logic | Expected Outcome (Success Pattern) |
| :--- | :--- | :--- |
| `test_node_init_style` | Ensures `node` is accepted as an argument and stored in `self.node`. **Strictly forbids type hints** to prevent regex failure. | `def __init__(self, node, ...)` AND `self.node = node` |
| `test_parameter_logic` | Checks if parameters are first **declared** and then accessed via the `.value` attribute. | `declare_parameter` AND `.value` |
| `test_topic_integrity` | Prevents the LLM from accidentally changing topic names or swapping messages (e.g., Dispatch vs. Feedback). | `create_subscription` with `/kcl_rosplan/action_feedback` |
| `test_qos_latching` | Verifies that the LLM recognized the `latch=True` requirement and translated it to QoS. | Presence of `TRANSIENT_LOCAL` |
| `test_interface_params` | Ensures the original data flow (passing `url` and `prefix`) is preserved instead of just passing the node handle. | `DomesticOntologyInterface(url, prefix)` |
| `test_clock_api` | Checks for the proper use of the Node-bound clock for timestamps. | `self.node.get_clock().now()` |
| `test_no_legacy` | Ensures absolute cleanup of ROS 1 symbols and outdated keywords. | Absence of `import rospy` and `latch=True` |

### Expected Outcome
- **High Performance:** The LLM must produce code that is logically sound (Subscribing to the right topic) and syntactically compliant with the style guide (No `: Node` type hints).
- **Behavioral Fidelity:** The derived states must be able to function exactly as they did in ROS 1, meaning the `KBInterface` must be initialized with the correct values fetched from the ROS 2 Parameter Server.
