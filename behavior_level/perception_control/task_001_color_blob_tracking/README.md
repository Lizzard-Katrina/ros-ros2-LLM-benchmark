# Task 001 – Color Blob Tracking (Perception → Control)

## 1. Brief Description

This task evaluates whether an LLM can correctly translate a ROS1 perception–control node into ROS2 while preserving **behavior-level semantics**.

The original ROS1 implementation:

- Subscribes to a camera image topic  
- Scans the image to detect a white color blob (RGB triplet check)  
- Computes the horizontal position of the blob  
- Applies a 3-region control policy (left / center / right)  
- Sends velocity commands through a service (`DriveToTarget`)  

The ROS2 version must preserve this **closed-loop behavior**:
> Perception → Decision → Control Output

This benchmark does **not** require byte-level translation.  
It validates **semantic equivalence** between ROS1 and ROS2 implementations.

---
source code file:
```https://github.com/apresland/autonomous-mobile-robots/blob/main/P2_Perception/src/ball_chaser/src/process_image.cpp```
---

## 2. Why We Hollowed This File

We hollowed the **perception + decision logic inside `process_image.cpp`** because:

- It contains the **core perception–control coupling**
- It represents a complete behavioral loop
- It is small enough to isolate, but complex enough to test reasoning
- It includes:
  - Image buffer iteration
  - RGB triplet detection
  - Geometric reasoning (image width / step)
  - Multi-branch decision logic
  - Service-based control output

We intentionally removed the internal logic while preserving:

- ROS2 node scaffolding
- Subscription interface
- Service client interface

This forces the LLM to reconstruct:

- The perception algorithm
- The region-based control policy
- The stop behavior when no blob is found
- The request sending pipeline

We avoid hollowing trivial wrappers or pure boilerplate to ensure the benchmark measures **behavioral reconstruction**, not syntax memorization.

---

## 3. Oracle Testcases Explanation

Each oracle test validates one **semantic component** of the original ROS1 behavior.  
All tests use regex / static matching only (no compilation or runtime).

---

### ✅ test_01_ros2_not_ros1_and_has_core_headers

**Why this test exists:**

We must ensure the translation is truly ROS2, not a disguised ROS1 copy.

**What it checks:**

- Includes `rclcpp/rclcpp.hpp`
- Includes `sensor_msgs/msg/image.hpp`
- Does NOT include `ros/ros.h`
- Does NOT use `ros::` APIs

**Expected outcome to pass:**

The implementation uses ROS2 APIs (`rclcpp`) exclusively.

---

### ✅ test_02_node_spin_and_image_subscription_pipeline

**Why this test exists:**

The behavioral loop requires:
- Node initialization
- Active spinning
- Image subscription with callback

Without this, perception cannot occur.

**What it checks:**

- `rclcpp::init`
- `rclcpp::spin`
- `rclcpp::shutdown`
- `create_subscription<sensor_msgs::msg::Image>`
- Topic name contains `"image"`
- A callback is attached (bind or lambda)

**Expected outcome to pass:**

The node subscribes to the image topic and spins properly.

---

### ✅ test_03_drive_service_client_request_fields_and_send

**Why this test exists:**

The control output in ROS1 was sent through `DriveToTarget`.

To preserve semantic equivalence, ROS2 must:
- Create a client
- Populate request fields
- Send the request

**What it checks:**

- `create_client<DriveToTarget>`
- Assignment to BOTH:
  - `linear_x` (or equivalent)
  - `angular_z` (or equivalent)
- `async_send_request`

**Expected outcome to pass:**

The node sends velocity commands through the DriveToTarget service.

---

### ✅ test_04_perception_scans_image_data_buffer

**Why this test exists:**

The ROS1 algorithm scans the image buffer.  
It does NOT check a single fixed pixel.

This ensures the perception step is reconstructed.

**What it checks:**

- Access to `msg.data[...]`
- A `for` loop reading from the data buffer

**Expected outcome to pass:**

The implementation iterates over image data.

---

### ✅ test_05_rgb_triplet_check_present

**Why this test exists:**

The ROS1 logic detects a white blob using:


with a combined condition.

This ensures semantic preservation of RGB-based detection.

**What it checks:**

- Access to `data[i+1]`
- Access to `data[i+2]`
- A condition using `&&` across channels

**Expected outcome to pass:**

Blob detection is based on RGB triplet logic.

---

### ✅ test_06_three_region_left_center_right_decision_with_geometry_thresholds

**Why this test exists:**

The original control policy divides the image into:

- Left third → turn left
- Right third → turn right
- Middle third → go forward

This geometric reasoning is central to behavior equivalence.

**What it checks:**

- `if / else if / else` structure
- Usage of `width` or `step`
- Two threshold-like cutpoints (≈ 1/3 and 2/3)

**Expected outcome to pass:**

The decision logic splits the image horizontally into three regions.

---

### ✅ test_09_stop_and_motion_semantics_without_literal_numbers

**Why this test exists:**

We must ensure:

1. The node detects blob presence or absence
2. The robot moves when blob is found
3. The robot stops when blob is not found

This prevents trivial solutions like:

- Always stop
- Always turn
- Always move forward

**What it checks:**

Detection indicator exists:
- `found`
- `pixel`
- `blob`
- sentinel `-1`
etc.

Motion evidence:
- Linear component assigned
- Angular component assigned with BOTH signs (+ and -)

Stop evidence (any of the following):
- Explicit assignment to 0
- `(0, 0)` helper call
- Implicit zero-initialized velocities copied into request

**Expected outcome to pass:**

- Both forward and turning behaviors exist
- Stop behavior exists
- Behavior depends on detection result

---

## 4. Summary

This benchmark validates:

- Structural correctness (ROS2 APIs)
- Perception reconstruction (RGB scan)
- Geometric reasoning (3-region policy)
- Control output pipeline (service request)
- Behavioral completeness (move + turn + stop)

It does **not** require identical syntax to ROS1.  
It requires preservation of **behavior-level semantics**.

This makes the benchmark suitable for evaluating:

> LLM capability in translating robotics behavior across ROS versions while maintaining control logic integrity.
