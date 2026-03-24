# Task 012: IMM-UKF-PDA Tracker ROS 2 Migration

## 1. Brief Description
This task involves migrating the core tracking logic of the **IMM-UKF-PDA (Interacting Multiple Model - Unscented Kalman Filter - Probabilistic Data Association)** node from ROS 1 to ROS 2 (Humble/Foxy). 

The node is a critical component of the perception pipeline, responsible for:
* **Coordinate Transformation**: Aligning sensor-frame detections with the global tracking frame using TF2.
* **Temporal Management**: Calculating precise time deltas ($dt$) between asynchronous sensor messages.
* **Filter Pipeline**: Executing the prediction, data association, and state update cycles for multiple tracked targets.
* **State Management**: Handling the lifecycle of tracked objects (Initial, Stable, Lost, or Dead).

The migration challenges the model to handle complex TF2 buffer lookups, ROS 2 specific message namespaces (`::msg::`), and high-precision timer/timestamp APIs.
---
source code file:
```https://github.com/autowarefoundation/autoware_ai_perception/blob/master/imm_ukf_pda_track/nodes/imm_ukf_pda/imm_ukf_pda.cpp```

---

## 2. Hollowing Strategy
The hollowing strategy follows a **"Dual-Core Logic Gap"** approach, removing two interconnected but distinct functional blocks to test different aspects of ROS 2 migration:

1.  **Spatial Transformation Block (`updateNecessaryTransform`)**:
    * **Hollowed Logic**: The entire process of looking up transforms between `input_frame`, `tracking_frame`, and `map_frame`.
    * **Goal**: To test if the model can switch from the synchronous `tf::TransformListener` to the asynchronous, exception-mandated `tf2_ros::Buffer` API.
    * **Constraints**: Pointer-based access (`tf_buffer_->`) and strict `try-catch` requirements are enforced via TODO comments.

2.  **Control Flow Pipeline (`tracker`)**:
    * **Hollowed Logic**: The main loop that manages timestamps, calculates $dt$, and orchestrates the UKF mathematical steps.
    * **Goal**: To evaluate the model's ability to handle ROS 2 `SharedPtr` callback signatures and the `rclcpp::Time` API.
    * **Constraints**: Precise calculation of $dt$ and preservation of the `prediction -> association -> update` sequence.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle tests utilize pattern matching to verify semantic correctness and API compliance without requiring a full build environment.

### Test 1: `test_tf2_buffer_pointer_usage`
* **Design Intent**: Verifies the use of the ROS 2 `tf2_ros::Buffer` pointer as defined in the node's member variables.
* **Expected Outcome**: The code must use `tf_buffer_->lookupTransform(...)`. Matches using `.` or legacy `tf_listener` will fail.

### Test 2: `test_tf2_time_lookup_accuracy`
* **Design Intent**: Checks for "Temporal Correctness" in spatial lookups—a common pitfall where developers use `TimePointZero` instead of the actual message timestamp.
* **Expected Outcome**: Presence of `input.header.stamp` (or the ROS 2 equivalent) inside the `lookupTransform` arguments.

### Test 3: `test_tf2_geometry_msgs_include`
* **Design Intent**: Detects missing dependency headers that cause obscure template errors in ROS 2.
* **Expected Outcome**: The file must contain `#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>` to support `tf2::fromMsg`.

### Test 4: `test_namespace_full_compliance`
* **Design Intent**: Ensures the model correctly adheres to the ROS 2 message naming convention, which is the most frequent cause of compilation failure.
* **Expected Outcome**: All instances of `DetectedObjectArray` must be prefixed with `autoware_msgs::msg::`.

### Test 5: `test_ukf_pipeline_integrity`
* **Design Intent**: Ensures the migration didn't accidentally skip or rearrange the core mathematical steps of the UKF algorithm.
* **Expected Outcome**: Explicit calls to `.prediction()`, `probabilisticDataAssociation()`, and `.update()` must exist in the correct logical flow.

### Test 6: `test_no_ros1_symbols`
* **Design Intent**: Acts as a "Legacy Cleaner" to ensure no ROS 1 artifacts are left behind.
* **Expected Outcome**: Total absence of `ros::Time`, `ros::Duration`, `.toSec()`, or `tf::TransformListener`.
