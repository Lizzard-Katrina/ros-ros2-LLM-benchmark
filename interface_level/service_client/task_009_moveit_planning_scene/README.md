# Task 009 — MoveIt: Apply Planning Scene Service

## 1. Brief Description

This task evaluates whether a model can correctly translate and reconstruct
a MoveIt Planning Scene manipulation workflow in ROS2.

The focus is on **semantic equivalence**, not just API replacement.
In particular, the task centers on correctly updating the planning scene when:

- Attaching an object to the robot (world → robot transition)
- Detaching an object from the robot (robot → world transition)

The model must understand MoveIt planning scene diffs and object lifecycle.


---
Source reference:
`https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html`

## 2. Why the Source Code is Hollowed Out

The function

`step_attach_object_and_remove_from_world__TODO(...)`


is intentionally hollowed out.

### Goal

Force the model to:

- Understand the tutorial context
- Reconstruct correct planning scene diff logic
- Preserve semantic transitions between world and robot state
- Avoid shallow or template-style translations

This prevents trivial copying and tests real comprehension of MoveIt logic.


---

## 3. Oracle Test Design (4 Failure Categories)

The oracle tests are static (regex-based) and do not require compilation.

They are grouped into **4 semantic categories**.


---

### Category 1 — CONTEXT

**Design Rationale**

Ensure the translation is truly ROS2 + MoveIt based,
not a mixed or partially migrated version.

**Checks**

- Uses `rclcpp`
- No ROS1 remnants (`ros::init`, `NodeHandle`, ROS_* macros)
- Includes MoveIt PlanningScene-related messages
- Publishes to `"planning_scene"` topic

**Expected Translation Outcome**

A proper ROS2 MoveIt node that publishes planning scene diffs.


---

### Category 2 — CORE TRANSITION (World → Attached)

**Design Rationale**

Verify the key semantic action:
removing the object from the world and attaching it to the robot
in the same diff.

**Checks**

- Constructs `PlanningScene`
- Creates a `CollisionObject`
- Uses `CollisionObject::REMOVE`
- Pushes removal into `world.collision_objects`
- Pushes `attached_object` into `robot_state.attached_collision_objects`
- Publishes diff and uses prompt boundary

**Expected Outcome**

Correct world → robot transition in one coherent planning scene diff.


---

### Category 3 — MINIMAL DIFF HYGIENE

**Design Rationale**

Many weak translations accidentally send oversized diffs
or leave stale objects.

We enforce minimal and clean diffs.

**Checks**

- `robot_state.is_diff = true`
- Explicit reset of:
  - `world.collision_objects`
  - `robot_state.attached_collision_objects`
- No re-adding attached object back into world

**Expected Outcome**

A minimal, clean planning scene diff
that only contains intended changes.


---

### Category 4 — DUALITY (Detach is Inverse)

**Design Rationale**

Attach and detach should form a semantic pair.
This checks deeper understanding of object lifecycle.

**Checks**

Detach step must:

- REMOVE from robot
- Push detach message to robot_state
- Return object to world

**Expected Outcome**

Correct robot → world transition
mirroring the attach step.


---

## Summary

This task evaluates:

- MoveIt planning scene semantics
- Object lifecycle understanding
- Diff-based scene updates
- Context-aware ROS1→ROS2 migration

A correct solution requires more than syntax translation —
it requires understanding how MoveIt manages world and attached objects.

