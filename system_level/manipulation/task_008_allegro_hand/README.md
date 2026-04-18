# Benchmark Task: Allegro Hand CAN-ROS System Integration

## 1. Brief Description
This task implements the core control and communication bridge for the **Wonik Robotics Allegro Hand** (a 16-DOF robotic manipulator). The goal is to evaluate an AI's ability to handle **System-Level Integration**, which requires maintaining data consistency across a low-level C++ CAN driver (`AllegroHandDrv`) and a high-level ROS2 controller node (`AllegroNode`).

The task focuses on:
* **Hardware Protocol Parsing**: Reconstructing 16-bit joint data from 8-bit CAN frames.
* **Physical Unit Scaling**: Converting raw encoder ticks to SI units (radians).
* **Real-time Synchronization**: Managing "Data Ready" flags (bitmasks) to ensure the control loop only runs on fresh sensor data.
* **Safety Interlocks**: Implementing an emergency stop that shuts down the ROS2 node upon hardware failure.

---
source code file
```https://github.com/Wonikrobotics-git/allegro_hand_ros_v5/blob/master-4finger/src/allegro_hand_controllers```

## 2. Implementation Logic (The "Holes")

### Task 1: CAN Data Unpacking (`_parseMessage`)
* **The Problem**: The hand sends joint data in compressed CAN frames. Each frame contains data for one finger (4 joints).
* **Logic Requirements**:
    * **ID Mapping**: Extract the finger index from the CAN ID using bitwise AND: `int findex = (id & 0x07)`.
    * **Bitwise Assembly**: Reconstruct 4 joint positions. Each is a 16-bit short split into two bytes. The required format is `data[i] | (data[i+1] << 8)`.
    * **Scaling**: For non-V4 hands, the raw value must be multiplied by `0.088` (to get degrees) and then converted to radians using `M_PI / 180.0`.
    * **State Flag**: Update the `_curr_position_get` bitmask using `(0x01 << findex)` to signal that this finger's data is now available.

### Task 2: Torque-to-PWM Conversion (`_writeDevices`)
* **The Problem**: High-level double torques must be converted to hardware PWM integers.
* **Logic Requirements**:
    * **Saturation**: Clamp input torques to a safe range (typically `±240.0`) to prevent hardware damage.
    * **Hardware Variants**: Detect "Type-A" hands and apply a `0.5x` scaling factor to specific joints (1, 5, and 9) due to different mechanical gear ratios.

### Task 3: Control Loop Synchronization (`updateController`)
* **The Problem**: The high-frequency control loop must stay synchronized with the asynchronous CAN bus.
* **Logic Requirements**:
    * **Safety Check**: Monitor `readCANFrames()`. If it returns a negative value, the node must call `rclcpp::shutdown()` immediately.
    * **State Management**: Before updating `current_position`, copy it to `previous_position`. This is mandatory for the finite difference calculation of velocity.
    * **Velocity Derivation**: Calculate `current_velocity` as `(current_position - previous_position) / dt`.
    * **Handshake Reset**: Crucially, call `canDevice->resetJointInfoReady()` at the end of the update block to reset the bitmask for the next cycle.

---

## 3. Testcase Design & Expected Outcomes

The Oracle Testcases are designed to detect "System-Level" failures where code might compile but fail in a real robotics environment.

| Testcase | Design Strategy | Expected Outcome (Pass Criteria) |
| :--- | :--- | :--- |
| **`test_can_unpack_logic`** | **Regex-based Math Check**: Verifies the bit-shift and scaling constants. | Matches `data[...] \| (data[...] << 8)` and the `0.088` constant. |
| **`test_bitmask_update`** | **State Machine Check**: Ensures the bitmask update uses the correct finger index. | Presence of `_curr_position_get \|= (0x01 << findex)`. |
| **`test_control_sync_flow`** | **Sequence Validation**: Ensures "Read -> Compute -> Reset" order. | Calls to `getJointInfo`, `computeDesiredTorque`, and `resetJointInfoReady` exist in sequence. |
| **`test_velocity_calculation`** | **Physics Derivation**: Checks for correct implementation of $v = \frac{dx}{dt}$. | Matches `(current_position[i] - previous_position[i]) / dt`. |
| **`test_emergency_stop`** | **Safety Interlock**: Checks if the node shuts down properly in ROS2. | Detects `if (status < 0)` followed by `rclcpp::shutdown()`. |
| **`test_data_backup`** | **Temporal Consistency**: Checks if old data is saved before being overwritten. | Finds `previous_position[i] = current_position[i]` before the new data fetch. |
| **`test_naming_consistency`** | **API Integrity**: Ensures LLM uses existing header variables, not "hallucinated" ones. | Correct use of `HAND_TYPE_A` (bool) instead of custom strings like `_hand_type`. |

---
