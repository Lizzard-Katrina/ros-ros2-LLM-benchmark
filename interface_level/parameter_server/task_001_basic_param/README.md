# Task 001 — Parameter Server Cache Semantics (ROS1 → ROS2)

## 1. Brief Description

This task evaluates semantic preservation when migrating ROS1 parameter-server client logic to ROS2.

The ROS1 source implements a subscription-backed parameter cache that:

- retrieves parameters with optional caching,
- subscribes to parameter updates from the master,
- updates the local cache on remote changes,
- invalidates parent namespace caches to preserve hierarchical consistency.

The ROS2 target is expected to reproduce these semantics using ROS2 parameter mechanisms (e.g., parameter services and parameter events), rather than ROS1 XMLRPC-based master calls.

This task focuses on parameter consistency semantics, not transport-level compatibility.

---
source code:
`https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/param.cpp`


## 2. Code Blanking Strategy

Instead of blanking many small helpers, this task blanks one coherent semantic loop:

- `getImpl(...)`
- `update(...)`
- `invalidateParentParams(...)`

These three functions together form the full cache-consistency cycle in ROS1:

cached get → subscription → remote update → cache refresh → parent invalidation

Other parts of the file remain intact so that:

- key resolution logic is preserved,
- data structures (cache map, subscribed set) remain visible,
- the model must reconstruct the behavioral semantics, not scaffolding.

---

## 3. Test Case Semantics

The oracle performs static semantic checks.  
It does not execute ROS nodes.

It verifies that the ROS2 solution preserves the *behavioral intent* of the ROS1 implementation.

---

### Test A — ROS2 parameter system usage

**Checks**
- Uses ROS2 APIs (`rclcpp`)
- Does not use ROS1 master/XMLRPC calls

**Expected Outcome**
The solution uses ROS2 parameter mechanisms rather than ROS1 infrastructure.

---

### Test B — Cache + subscription state

**Checks**
- Presence of a cache container (map/unordered_map)
- Presence of a subscribed-key set
- Use of mutex/locking

**Expected Outcome**
The solution maintains local state similar to ROS1 cached parameters.

---

### Test C — Real parameter event subscription

**Checks**
- Uses `ParameterEvent` or `ParameterEventHandler`
- Actually constructs a subscription/handler object

**Expected Outcome**
The solution replaces ROS1 `subscribeParam` with ROS2 event-driven updates.

---

### Test D — update() semantics

**Checks**
- update() locks shared state
- update() checks whether a key is subscribed before updating cache
- update() mutates cache
- update() calls invalidateParentParams()

**Expected Outcome**
Only subscribed parameters influence the cache, matching ROS1 gating logic.

---

### Test E — invalidateParentParams() semantics

**Checks**
- Iterates over parent namespaces
- Computes parent keys
- Erases parent entries from cache

**Expected Outcome**
Hierarchical parameter consistency is preserved.

---

### Test F — getImpl() semantics

**Checks**
- Branches on `use_cache`
- Attempts cache lookup
- Registers interest/subscription on first access
- Has a remote query path using ROS2 parameter APIs

**Expected Outcome**
The function supports both cached and uncached retrieval paths.

---

## Summary

This task measures whether a model can reconstruct ROS1’s subscription-backed parameter cache semantics using ROS2 parameter mechanisms.

It emphasizes:

- event-driven cache updates,
- hierarchical invalidation,
- subscription-aware caching behavior.
