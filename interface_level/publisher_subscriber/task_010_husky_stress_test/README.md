# task_010_husky_stress_test

## Task Overview

This task evaluates an LLM’s ability to translate **ROS1 interface-level publisher/subscriber semantics**
to ROS2, focusing on **implicit, high-frequency sensor publishers** defined across launch and URDF files.

The task is designed as an **interface-level correctness benchmark** and does not require
building, running, or simulating a ROS2 system.


---

## Source Files (ROS1)

The task is derived from the Husky simulator (Clearpath Robotics).
source:
```https://github.com/husky/husky_simulator```
### 1. Launch File

**Original location:**
```husky_simulator/husky_gazebo/launch/spawn_husky.launch```


**Why this file is used:**

This launch file defines a complete interface pipeline:

- Generation of `robot_description` via Xacro
- Consumption of this description by `robot_state_publisher`
- Spawning of the robot entity using Gazebo

Although no explicit publishers are defined here, this file orchestrates
**the existence and wiring of downstream publishers**.


---

### 2. URDF / Xacro File

**Original location:**

```husky_simulator/husky_gazebo/urdf/husky.gazebo.xacro```


**Why this file is used:**

This file defines **implicit ROS publishers** through Gazebo plugins:

- IMU sensor publishing at high frequency
- GPS / NavSat publishers on multiple topics

These publishers do not appear as ROS nodes in launch files,
making them a common failure point in ROS1 → ROS2 migration.


---

## What Is Extracted (“挖空”)

Only interface-level logic is retained:

### From `spawn_husky.launch`
- Robot description generation pipeline
- robot_state_publisher semantics
- Robot spawn / registration step

### From `husky.gazebo.xacro`
- IMU publisher semantics
- GPS / NavSat publisher semantics
- High-frequency publishing intent

Simulation details, controller configuration, and numeric parameters are removed.


---

## LLM Task

The LLM is asked to:

- Translate ROS1 launch + URDF-based interfaces to ROS2
- Preserve semantic intent, not syntax
- Model multiple high-frequency sensor publishers
- Avoid ROS1 APIs entirely

The output does **not** need to be executable.


---

## Oracle Test Design


The oracle tests verify **interface-level correctness and preservation of key semantics**, without requiring ROS2 runtime execution.

### Test Group 0: File existence
- **Purpose:** Ensure LLM output contains both translated files.
- **Expected Outcome:** Both files exist in `submission/`. Missing file → FAIL.

### Test Group 1: ROS1 artifact removal
- **Purpose:** Confirm no ROS1-specific syntax remains in translated launch.
- **Checked Patterns:** `<node pkg=`, `$(find`, `rosparam`, `rostopic`, `launch`.
- **Expected Outcome:** None of these patterns appear. If any exist → FAIL.

### Test Group 2: Robot description interface
- **Purpose:** Ensure URDF/Xacro robot description is translated correctly and referenced in launch.
- **`test_robot_description_defined`**  
  - **Checks:** `robot`, `link`, `joint` keywords present in `husky.gazebo.xacro`.
  - **Expected Outcome:** All keywords exist.
- **`test_robot_description_consumed_in_launch`**  
  - **Checks:** `robot_description` parameter is referenced in launch.
  - **Expected Outcome:** Launch file contains `robot_description`.

### Test Group 3: IMU / GPS publisher semantics
- **Purpose:** Ensure sensor plugins/interfaces (IMU/GPS) are preserved in translation.
- **IMU Check Keywords:** `imu`, `inertial`, `sensor`, `imu_controller`
- **GPS Check Keywords:** `gps`, `navsat`, `fix`, `gps_controller`
- **Expected Outcome:** At least one keyword for each sensor present in `husky.gazebo.xacro`.

### Test Group 4: High-frequency / update rate semantics
- **Purpose:** Confirm that high-frequency publisher semantics are retained.
- **Keywords Checked:** `updateRate`, `frequency`, `hz`, `publish_rate`
- **Expected Outcome:** At least one keyword present in `husky.gazebo.xacro`.

---

## 3. Oracle Test Philosophy

1. **Interface-level only:** The oracle does not require running ROS2, Gazebo, or launch files. It evaluates the **structure and semantics** of translation output.
2. **Weak matching:** Keywords are used to identify preserved functionality; exact syntax or plugin names may differ due to LLM creativity.
3. **Coverage:** Tests ensure:
   - LLM recognizes which files to translate
   - ROS1 artifacts are removed
   - Core publisher/subscriber interfaces are preserved
   - Sensor semantics (IMU/GPS) and high-frequency properties are retained

This ensures the benchmark focuses on **semantic fidelity in interface-level migration**, aligning with the task goal.

## Run tests inside ros2 ws

``` python3 -m pytest src/task_010/test/test_oracle_ros2.py```
