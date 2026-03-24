# Task: Pick-and-Place Manipulation (Franka Panda)

## Category
System-Level Manipulation — Integration Correctness

## Source
Derived from the ROS1 project:
https://github.com/elena-ecn/pick-and-place

## Task Goal
This benchmark evaluates whether an LLM can correctly translate
and reconstruct **system-level manipulation logic** when migrating
from ROS1 to ROS2.

The task focuses on **integration correctness**, not low-level
perception or motion planning algorithms.

---

## System Integration Flow

Camera  
→ Object Detection  
→ Object Publication  
→ State Machine Decision  
→ Manipulation Controller  
→ Pick  
→ Place  

---

## TODO Breakdown and Expected Outcomes

### 1. `object_detector_imagecall_todo.py`
**Source directory:** `pick_and_place/scripts/`

**Integration focus:**  
Perception trigger → internal detection pipeline

**TODO description:**  
Trigger the object detection pipeline when a camera image is received.
The implementation should correctly update internal detection results
using existing helper functions.

**Expected outcome:**  
- The image callback activates the perception pipeline
- Internal object buffers are updated accordingly

---

### 2. `object_detector_publish.py`
**Source directory:** `pick_and_place/scripts/`

**Integration focus:**  
Perception → System message interface

**TODO description:**  
Construct and publish a `DetectedObjectsStamped` message
based on current perception results.

**Expected outcome:**  
- Detected objects are published to `/object_detection`
- Message fields (pose, size, color) are correctly populated

---

### 3. `place_state_machine_on_enter_todo.py`
**Source directory:** `pick_and_place/scripts/`

**Integration focus:**  
System-level behavior sequencing

**TODO description:**  
Implement the core state-entry logic that:
- selects an object
- triggers the pick-and-place routine
- advances the state machine correctly

**Expected outcome:**  
- State transitions follow the intended pick-and-place loop
- The system progresses from selection to execution and back to home

---

### 4. `controller.py`
**Source directory:** `pick_and_place/scripts/`

**Integration focus:**  
Decision → Manipulation execution

**TODO description:**  
Execute the complete pick-and-place routine for a selected object,
using existing robot and gripper interfaces.

**Expected outcome:**  
- Pick motion is executed at the object pose
- Place motion is executed at the correct bin
- The manipulation sequence completes successfully

---

## Evaluation Criterion

This task is considered successful if the translated ROS2 code:
- Preserves system-level data flow
- Executes correct state transitions
- Triggers pick-and-place actions in the correct order

Algorithmic optimality is **not** evaluated.
