# Task 011 — Camera Reconfigure Client (ROS1 → ROS2)

## 1. Brief Description

This task evaluates semantic preservation when migrating a ROS1 camera configuration client to ROS2.

The source program uses a ROS1 dynamic_reconfigure client targeting the camera driver namespace `head_camera/driver`. It provides two user-facing behaviors:

- enable automatic exposure and white balance
- disable automatic exposure and white balance

The ROS2 target is expected to preserve the same configuration semantics using ROS2-native mechanisms (parameters / parameter services), while keeping the same target namespace intent.

---

## 2. Code Blanking Strategy

The original file is a minimal interface-level client. Blanking therefore focuses on the semantic core rather than boilerplate:

- `CameraReconfigure.__init__` was blanked to remove the configuration interface initialization.
- `CameraReconfigure.enable_auto` and `CameraReconfigure.disable_auto` were blanked to remove the key configuration operations.

The CLI entrypoint and high-level control flow remain intact to preserve the original invocation semantics (enable vs disable) while forcing reconstruction of the ROS2-side interface behavior.

---

## 3. Test Case Semantics

The oracle checks whether the ROS2 solution preserves the intended behavior of the ROS1 source.  
It does not enforce formatting, logging style, or code organization beyond semantic constraints.

### Test A — ROS2-only implementation (no ROS1 dependencies)
**Checks**
- The solution imports and uses ROS2 Python (`rclpy`).
- The solution does not depend on ROS1-only APIs (e.g., `rospy`) or ROS1 dynamic_reconfigure.

**Expected outcome**
A correct solution uses ROS2 mechanisms rather than retaining ROS1 client libraries.

---

### Test B — Target namespace preservation
**Checks**
- The solution references the semantic target namespace `head_camera/driver`.

**Expected outcome**
A correct solution configures the same intended camera driver target as the ROS1 source, not a different or generic node.

---

### Test C — ROS2 parameters mechanism is used
**Checks**
- The solution uses ROS2 parameter-setting mechanisms (e.g., `set_parameters`, parameter client/service patterns).

**Expected outcome**
A correct solution implements configuration changes via ROS2 parameters rather than ROS1 dynamic_reconfigure.

---

### Test D — Initialization establishes a configuration interface
**Checks**
- `__init__` creates or holds a ROS2 node context.
- `__init__` sets up a mechanism to apply configuration to the target driver node (parameters / parameter client / service).

**Expected outcome**
A correct solution can actually perform configuration updates to the target using ROS2 semantics.

---

### Test E — disable_auto semantics
**Checks**
- `disable_auto` assigns both:
  - `auto_exposure = False`
  - `auto_white_balance = False`

**Expected outcome**
Calling `disable_auto()` disables both automatic functions, matching ROS1 behavior.

---

### Test F — enable_auto semantics
**Checks**
- `enable_auto` assigns both:
  - `auto_exposure = True`
  - `auto_white_balance = True`

**Expected outcome**
Calling `enable_auto()` enables both automatic functions, matching ROS1 behavior.

---

## Summary

This task measures semantic migration of runtime configuration behavior:

- correct target selection (`head_camera/driver`)
- correct state changes (True/False for the two parameters)
- correct ROS2 interface mechanism (parameters), avoiding ROS1 dependency retention
