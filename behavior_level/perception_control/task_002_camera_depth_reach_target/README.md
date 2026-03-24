# Task 002 – Camera Depth Reach Target (MoveIt Pick & Place Baseline)

## 1. Brief Description

This task evaluates whether an LLM can translate a ROS1 MoveIt pick-and-place pipeline into ROS2 while preserving **behavior-level semantics**.

The original ROS1 tutorial implements a complete manipulation loop:

- Add collision objects to the planning scene
- Define grasp configurations
- Execute a pick action
- Define place configurations
- Execute a place action

This task focuses on reconstructing the **manipulation behavior structure**, not numerical tuning.

It belongs to the **Perception → Control (Manipulation)** category, where scene modeling and motion planning produce structured robot actions.

It serves as a baseline which should yeild a high (>90%) passing rate
---

Reference tutorial:
```https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/perception_pipeline/perception_pipeline_tutorial.html```
---

## 2. Why We Hollowed These Parts

We hollowed three behavior-critical regions of the ROS1 source:

### (A) `addCollisionObjects(...)`

This function defines the task scene:

- Support surface for pick
- Support surface for place
- Graspable object

We hollowed it to test whether the LLM can:

- Reconstruct scene modeling logic
- Define collision objects with unique IDs
- Add them properly to the planning scene

We did **not** hollow node scaffolding or MoveGroup initialization to avoid testing trivial ROS2 boilerplate.

---

### (B) `pick(...)`

This function encodes the manipulation semantics:

- Grasp pose
- Approach motion
- Retreat motion
- Gripper pre/post grasp posture

We hollowed this part because it captures:

- Structured motion configuration
- Object interaction semantics
- Multi-component grasp definitions

This prevents trivial “call pick() only” solutions.

---

### (C) `place(...)`

This mirrors pick but in reverse:

- Place pose
- Approach and retreat
- Gripper release posture

We hollowed this to ensure:

- Full pipeline reconstruction
- Behavioral completeness (pick + place)

---

## 3. Oracle Testcases Explanation

All oracle tests use static pattern matching (regex + string search).  
No compilation or execution is required.

Each test corresponds to one semantic component of the original ROS1 tutorial.

---

### ✅ test_01_ros2_not_ros1_and_core_headers

**Why:**  
Ensure true ROS2 translation (not ROS1 copy).

**Checks:**  
- Uses `rclcpp`
- Uses ROS2 message namespaces
- Does not include `ros/ros.h`

**Expected outcome:**  
Pure ROS2 APIs.

---

### ✅ test_02_node_spin_and_execution_pipeline

**Why:**  
The node must initialize, spin, and shut down properly.

**Checks:**  
- `rclcpp::init`
- `rclcpp::spin`
- `rclcpp::shutdown`

**Expected outcome:**  
Active ROS2 execution loop.

---

### ✅ test_03_collision_objects_added_to_scene

**Why:**  
The manipulation task requires scene modeling.

**Checks:**  
- Usage of `CollisionObject`
- Primitive shape definition
- ADD operation
- PlanningSceneInterface add/apply call

**Expected outcome:**  
Objects are constructed and added to the planning scene.

---

### ✅ test_04_pick_pipeline_present

**Why:**  
Ensures pick action is executed.

**Checks:**  
- `move_group.pick(...)`
- Grasp configuration defined

**Expected outcome:**  
Pick action is called with a grasp configuration.

---

### ✅ test_05_place_pipeline_present

**Why:**  
Ensures place action is executed.

**Checks:**  
- `move_group.place(...)`
- PlaceLocation configuration defined

**Expected outcome:**  
Place action is called with placement configuration.

---

### ✅ test_06_grasp_structure_components

**Why:**  
Prevents trivial or empty grasp definitions.

**Checks:**  
- `grasp_pose`
- `pre_grasp_approach`
- `post_grasp_retreat`
- `pre_grasp_posture`
- `grasp_posture`

**Expected outcome:**  
Structured grasp definition.

---

### ✅ test_07_place_structure_components

**Why:**  
Ensures structured placement configuration.

**Checks:**  
- `place_pose`
- `pre_place_approach`
- `post_place_retreat`
- `post_place_posture`

**Expected outcome:**  
Structured placement definition.

---

### ✅ test_08_scene_object_ids_and_support_surfaces

**Why:**  
Ensures semantic consistency with the original tutorial.

**Checks:**  
- Presence of support surfaces
- Object IDs referenced consistently in pick/place

**Expected outcome:**  
Logical linkage between scene and manipulation steps.

---

### ✅ test_09_stop_and_motion_semantics_without_literal_numbers

(If retained in this task)

**Why:**  
Ensures behavior completeness and prevents trivial solutions.

---

## 4. Task Position in Benchmark Suite

This task is classified as a:

> **Behavior-Level Manipulation Baseline Task**

It validates:

- Scene modeling
- Structured grasp configuration
- Structured placement configuration
- Pick-and-place action sequencing

It does not test numerical optimality or real execution correctness.

---

## 5. Intended Use

This task serves as:

- A structural translation baseline
- A sanity check for ROS1 → ROS2 manipulation pipeline reconstruction
- A minimal correctness threshold for manipulation tasks

It is not intended to strongly discriminate between advanced LLMs.
