# Task 008: Multi-mode Kinematics & Spatial Odometry Integration

## 1. Brief Description
This task evaluates the model's ability to implement complex robot kinematics and state estimation within a ROS 2 C++ driver. The focus is on the AgileX LIMO platform, requiring precise mathematical mapping between high-level velocity commands and low-level hardware constraints across multiple motion modes (Ackermann, Mecanum, and Four-wheel differential).
---
Source code file:
```https://github.com/agilexrobotics/limo_ros/blob/master/limo_base/src/limo_driver.cpp#L356```
---

## 2. Implementation Holes & Logic
The benchmark targets the mathematical core of the robot's locomotion system:

### A. Inverse Kinematics Mapping (`twistCmdCallback`)
* **Logic**: Converts `geometry_msgs/Twist` into hardware-specific steering and velocity commands. 
* **Challenge**: The model must correctly implement the Ackermann steering geometry, calculating angles based on the wheelbase and track while enforcing mechanical limit clamping.

### B. Forward Kinematics & Odometry (`publishOdometry`)
* **Logic**: Performs dead reckoning by integrating raw feedback into a global pose.
* **Challenge**: Requires a 2D rotation matrix projection to transform local body velocities ($v_x$, $v_y$) into global coordinates ($X$, $Y$) based on the current heading, scaled by the loop's time delta ($dt$).

---

## 3. Oracle Testcase Design (Hardcore Evaluation)
These 6 tests perform semantic analysis to ensure mathematical and physical consistency:

### 1. Ackermann Geometry (`test_ackermann_inverse_kinematics`)
* **Design**: Scans for trigonometric identities (e.g., `atan`, `tan`) linked to the wheelbase.
* **Expected Outcome**: Rejects simple linear assignments; requires correct geometric modeling of non-holonomic steering.

### 2. Steering Safety (`test_steering_limit_clamping`)
* **Design**: Validates mechanical constraint enforcement.
* **Expected Outcome**: The steering angle must be clamped by `max_inner_angle_` to prevent damage to the physical steering linkage.

### 3. Rotation Matrix Projection (`test_odom_integration_frames`)
* **Design**: Validates the spatial transformation logic.
* **Expected Outcome**: Ensures global $X$ and $Y$ updates use $\cos(\theta)$ and $\sin(\theta)$ projections of the body-frame velocities.

### 4. Holonomic Awareness (`test_mecanum_lateral_awareness`)
* **Design**: Checks for lateral velocity ($v_y$) integration.
* **Expected Outcome**: In Mecanum mode, the model must account for side-slip velocity in the odometry output.

### 5. Temporal Consistency (`test_time_differential_consistency`)
* **Design**: Verifies Euler integration scaling.
* **Expected Outcome**: All spatial displacements must be scaled by the time increment ($dt$) to maintain physical accuracy.

### 6. Protocol Serialization (`test_protocol_bit_shifting`)
* **Design**: Validates low-level byte-stream packing.
* **Expected Outcome**: Ensures 16-bit command integers are correctly serialized into 8-bit bytes using bit-shifting (`>> 8`) for serial communication.
