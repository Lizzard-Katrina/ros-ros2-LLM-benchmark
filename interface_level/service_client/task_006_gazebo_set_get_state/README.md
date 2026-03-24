# task_006_gazebo_set_get_state (ROS1 → ROS2)

## 1. Brief Description

This task evaluates ROS1→ROS2 translation quality for a Gazebo interaction node that:
- Subscribes to Gazebo state topics to track a model and a specific link pose.
- Calls a Gazebo service to set a model pose and twist.

The primary focus is **interface-level correctness**, especially the **ROS2 service client pattern** (service availability waiting, async request, spinning for completion, and response handling). Evaluation is performed via static source inspection only (regex/string matching), without compilation or runtime execution.

---
source code:
`https://gist.github.com/histvan95/261482184e36bb238d9c45a361586316`
## 2. What Was Removed (Holes) and Why

To isolate the ROS1→ROS2 migration skills and prevent solutions from succeeding via superficial edits, we remove (mask) the following blocks from the ROS1 source before presenting it to the model:

### (A) Service client declaration
**Removed:** the ROS1 `ros::ServiceClient` global.
**Rationale:** forces the solution to introduce the correct ROS2 client handle type and ownership pattern (e.g., `rclcpp::Client<...>::SharedPtr`), which is central to this task.

### (B) Service call implementation block (`set_model_state(...)`)
**Removed:** the entire function body that builds the request and performs the call.
**Rationale:** forces a correct ROS2-native service request workflow:
- build request fields correctly
- wait for service availability (retry semantics)
- send async request
- wait for completion via spinning
- check response success and log outcome

### (C) Node + subscriptions + service client creation
**Removed:** ROS1 init and creation of subscribers and service client in `main`.
**Rationale:** forces translation of ROS1 communication primitives to ROS2:
- node creation
- `create_subscription` usage (topic name + message type)
- `create_client` usage (service name + service type)

### (D) Main flow: pose/twist initialization + service invocation
**Removed:** pose/twist field assignments and the final call site.
**Rationale:** ensures the solution reconnects the pipeline end-to-end: construct pose/twist, then invoke the model-state update routine using task-specific target identifiers.

Non-core helper logic (e.g., `getIndex`) may remain unmasked to keep the task focused on ROS2 interfaces instead of general C++ algorithms.

---

## 3. Oracle Testcases (Design Rationale + Expected Outcome)

All oracle tests:
- operate on the produced source code only (regex + string search)
- do not compile or execute
- are independent (each validates one concept)
- check both presence of correct ROS2 concepts and absence of ROS1 residue where applicable

### Test 01 — ROS2 core usage + no ROS1 residue
**Design goal:** verify a true ROS2 translation rather than a partially edited ROS1 program.
**Expected outcome to pass:**
- The code uses ROS2 core (`rclcpp`) concepts.
- ROS1 constructs are absent (e.g., `ros::NodeHandle`, ROS1 logging macros, ROS1 `.call()` service invocation).

---

### Test 02 — Correct Gazebo service client type + endpoint
**Design goal:** ensure the solution targets the correct Gazebo service and correct service type for this task.
**Expected outcome to pass:**
- A ROS2 service client is created for the Gazebo SetModelState service type.
- The service endpoint string corresponds to `/gazebo/set_model_state`.

---

### Test 03 — Service availability retry with shutdown guard (strict)
**Design goal:** distinguish robust ROS2 client implementations from one-shot or hanging waits.
**Expected outcome to pass:**
- The code waits for service availability via `wait_for_service`.
- Service availability logic is retried in a loop (not a single check).
- The waiting logic includes an explicit ROS2 shutdown guard (e.g., `rclcpp::ok()`).

---

### Test 04 — Request populates all required model_state fields
**Design goal:** ensure the request is semantically correct and fully populated, not a skeleton request.
**Expected outcome to pass:**
- The request assigns `model_name`, `reference_frame`, `pose`, and `twist` into the service request’s `model_state` structure.

---

### Test 05 — Full async call chain + completion check + non-success handling (strict)
**Design goal:** enforce the canonical ROS2 service workflow and require explicit handling of failure/timeout completion paths.
**Expected outcome to pass:**
- The request is sent using an asynchronous client API.
- The solution waits for completion using a ROS2 spinning mechanism.
- Completion is checked explicitly against a success return code.
- A non-success path exists (e.g., timeout/failure handling branch).
- The response is retrieved from the future and a success indicator from the response is inspected.
- ROS2 logging macros are used for reporting outcomes.

---

### Test 06 — Task-specific subscriptions and name→pose indexing concept (combined)
**Design goal:** ensure the node still performs the “get state” portion using the correct Gazebo topics and message types, and that it conceptually maps a name list to a pose list.
**Expected outcome to pass:**
- Subscriptions exist for both Gazebo state topics:
  - `/gazebo/model_states` using the correct ROS2 message type
  - `/gazebo/link_states` using the correct ROS2 message type
- The implementation references the task-specific identifiers:
  - model name `ball`
  - link name `ball::body`
- The callback logic connects a `name` list to a `pose` list via indexing (semantic concept, not exact implementation).

---

### Test 07 — Callback signature uses SharedPtr message types (strict)
**Design goal:** encourage idiomatic ROS2 subscription callbacks and distinguish pass-by-value “ROS1-like” translations.
**Expected outcome to pass:**
- Callback parameters are expressed using ROS2 message `SharedPtr` forms for both ModelStates and LinkStates.

---

### Test 08 — Waits for future completion using spin/executor mechanism (flexible)
**Design goal:** ensure the solution uses an actual ROS2 waiting mechanism for the service future without enforcing a single coding style.
**Expected outcome to pass:**
- The solution uses a ROS2 spin-based mechanism to wait for the service future to complete (e.g., `rclcpp::spin_until_future_complete(...)` or an equivalent executor-based `spin_until_future_complete(...)` pattern).

---

## Notes

- This is an interface-level benchmark: correctness is judged by whether the produced code contains the intended ROS2 semantic concepts, not by runtime behavior in Gazebo.
- Minor formatting differences and variable naming differences are allowed as long as the required concepts are present.
