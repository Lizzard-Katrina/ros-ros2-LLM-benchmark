# Task 005 — ROS1 → ROS2 Perception-Control Pipeline Migration (Behavior-Level Correctness)

## 1. Brief Description

This task evaluates whether a Large Language Model (LLM) can correctly translate a complex ROS1 perception-control camera node into ROS2 **while preserving behavior-level semantics**, not just syntax.

The original ROS1 node implements a multi-stream camera driver with:

- Depth / Color / IR frame handling
- FrameSet-level processing
- Post-processing filters
- Optional depth-to-color alignment
- Point cloud generation
- Multi-threaded color decoding pipeline
- IMU message publishing
- Subscription-aware lazy execution

The LLM is required to migrate this system to ROS2 while preserving:

- Frame processing semantics
- Threading structure
- Pipeline ordering
- Subscription gating logic
- Alignment behavior
- Depth scaling semantics
- Queue-based producer-consumer architecture

This task does **not** require compilation or runtime execution.  
Correctness is validated via **pattern-based semantic oracle tests**.
---

source code file:
```https://github.com/orbbec/OrbbecSDK_ROS1/blob/v2-main/src/ob_camera_node.cpp```
---

## 2. Removed (Blanked) Regions and Rationale

We intentionally removed three major semantic blocks from the ROS1 source before translation.

The goal is to test whether the LLM can reconstruct the correct system-level behavior during migration.

---

### 🔹 (A) `onNewFrameSetCallback` — FrameSet Processing Loop

**Removed region:**  
The entire central FrameSet processing logic.

**Why this region was removed:**

This block is the core perception pipeline. It is responsible for:

- Retrieving frames from the FrameSet
- Applying post-processing filters
- Performing depth registration / alignment
- Dispatching frames either:
  - to a color processing queue (producer-consumer model), or
  - directly to point cloud publishing
- Forwarding non-color streams through a common handler

If this logic is reconstructed incorrectly, the system behavior changes significantly.

This block tests whether the LLM understands:

- Multi-stream synchronization
- Alignment semantics
- Frame mutation via `pushFrame`
- Conditional dispatch logic
- Iteration over stream types

---

### 🔹 (B) `onNewColorFrameCallback` — Color Thread Consumer Loop

**Removed region:**  
The FIFO consumer thread responsible for:

- Waiting on a condition variable
- Popping FrameSet from queue
- Decoding color frame
- Publishing point cloud
- Forwarding frame to `onNewFrameCallback`

**Why this region was removed:**

This tests whether the LLM can reconstruct:

- Proper producer-consumer synchronization
- Condition variable wait with predicate (not busy-wait)
- Correct FIFO ordering
- Correct semantic order:


This is critical for behavior-level correctness.

---

### 🔹 (C) `onNewFrameCallback` — Single Frame Publishing Logic

**Removed region:**  
Subscription-gated processing logic, flip branch, and depth scaling behavior.

**Why this region was removed:**

This block tests:

- Subscription-aware lazy processing
- Image / CameraInfo / Metadata gating
- Optional image flip branch
- Depth scaling semantics for DEPTH stream
- Correct publish path structure

If incorrectly reconstructed:

- The node may waste computation
- Depth images may be unscaled
- Flip configuration may be ignored

---

## 3. Oracle Tests — What Each Test Validates

All tests are **pattern-based semantic checks**.  
They validate concepts, not exact syntax.

Each test is independent.

---

### ✅ Test Group 1 — FrameSet Semantics

---

#### `test_frameset_retrieves_expected_frames`

**Validates:**
- Retrieval of multiple frame types from FrameSet:
  - Depth
  - Color
  - Left Color
  - Right Color
  - IR Left
  - IR Right

**Expected Outcome:**
The ROS2 code must retrieve all relevant frame types via `getFrame(...)` or equivalent.

---

#### `test_frameset_applies_filters_and_updates_frameset`

**Validates:**
- Application of:
  - `processDepthFrameFilter`
  - `processColorFrameFilter`
  - IR filters
- Reinsertion of processed frames using `pushFrame(...)`

**Expected Outcome:**
Processed frames must replace original frames in the FrameSet.

---

#### `test_frameset_alignment_semantics_present`

**Validates:**
- Presence of alignment logic
- Use of `align_filter_`
- Conditional alignment branch

**Expected Outcome:**
If alignment is enabled, the FrameSet must be passed through an alignment filter.

---

#### `test_frameset_dispatches_to_color_queue_or_falls_back_to_pointcloud_publish`

**Validates:**
- Producer logic:
  - `colorFrameQueue_.push(...)`
  - `notify_one()` or `notify_all()`
- Fallback behavior:
  - `publishPointCloud(...)`

**Expected Outcome:**
Frames must either be enqueued for color processing or directly published.

---

#### `test_frameset_forwards_non_color_streams_through_common_handler`

**Validates:**
- Iteration over IMAGE_STREAMS
- Forwarding non-color frames via `onNewFrameCallback`
- Optional IR MJPG decoding

**Expected Outcome:**
Non-color streams must still reach the common handler.

---

### ✅ Test Group 2 — Color Thread Semantics

---

#### `test_color_thread_uses_condition_variable_wait`

**Validates:**
- `condition_variable.wait(lock, predicate)` usage
- Shutdown-aware predicate

**Expected Outcome:**
Thread must not busy-wait.

---

#### `test_color_thread_consumes_fifo_and_processes_in_pipeline_order`

**Validates:**
- FIFO queue consumption:
  - `front()`
  - `pop()`
- Order of operations:
  - decode
  - publishPointCloud
  - onNewFrameCallback

**Expected Outcome:**
Color processing must preserve pipeline ordering.

---

### ✅ Test Group 3 — Single Frame Semantics

---

#### `test_single_frame_has_subscription_gate_for_image_camerainfo_metadata`

**Validates:**
- Subscription gating based on:
  - image publisher
  - camera_info publisher
  - metadata publisher

**Expected Outcome:**
Processing should occur only if at least one subscriber exists.

---

#### `test_single_frame_supports_flip_branch_and_depth_scaling_hook`

**Validates:**
- Optional image flip branch using `cv::flip`
- DEPTH-specific branch
- Presence of depth scaling semantics:
  - `getValueScale`
  - `depth_scale`
  - multiplication by scale

**Expected Outcome:**
Depth frames must incorporate scale conversion semantics.
Flip must conditionally modify image output.

---

### ✅ Test Group 4 — ROS2 Migration Correctness

---

#### `test_ros2_node_usage_and_no_ros1_leftovers`

**Validates:**
- Presence of ROS2 constructs:
  - `rclcpp`
  - ROS2 publishers/subscribers
- Absence of ROS1 constructs:
  - `ros::NodeHandle`
  - `ros::Time`
  - `ROS_INFO`
  - `ros::ok`

**Expected Outcome:**
No ROS1 APIs should remain in translated code.

---

## Strict vs Flexible Design

This task is part of a larger benchmark suite.

Some tasks use strict structural matching.
Others use more flexible semantic matching.

This task belongs to the **strict structural semantic category**, meaning:

- The translated ROS2 code must preserve pipeline structure.
- Alignment and depth scaling must not be omitted.
- Producer-consumer threading must remain intact.

---

## Summary

This task evaluates:

- Behavior-level equivalence
- Pipeline reconstruction ability
- Multi-thread semantic understanding
- Conditional logic preservation
- ROS1 → ROS2 API migration correctness

It is intentionally designed to detect:

- Silent semantic drops
- Missing alignment logic
- Incorrect threading reconstruction
- Depth scaling omission
- Incomplete stream forwarding

Passing this task implies the LLM preserved system-level perception-control semantics during migration.
