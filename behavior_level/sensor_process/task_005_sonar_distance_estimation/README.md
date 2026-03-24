# Task 005: Integrated 2D Kinematics and Sonar Perception

## 1. Brief Description
This task requires the implementation of the core physical and perceptual engine for the `turtlesim` node. It moves beyond simple command-following by forcing the AI to reconstruct a **Holonomic/Non-Holonomic Motion Model** and an **Analytical Sonar Sensor**. The turtle must update its pose based on velocity inputs and simultaneously "sense" its environment by calculating the precise Euclidean distance to window boundaries (walls) within a specific Field of View (FOV).
---
source code file
```https://github.com/ros/ros_tutorials/blob/rolling/turtlesim/src/turtle.cpp```

## 2. Excavation Strategy
We have performed a **Function-Level Excavation** within `Turtle::update`. The goal is to test the AI's ability to handle high-level robotics concepts without structural "scaffolding":

* **Logic Removal**: All pose-update math and distance-to-wall calculations have been stripped.
* **Geometric Abstraction**: The TODOs define physical constraints (e.g., "30-degree FOV", "Analytical Intersection") rather than providing formulas.
* **System Coupling**: The AI must correctly interface with the existing `turtlesim` architecture, including parameter lookups (for holonomic mode) and coordinate system inversions.

## 3. Oracle Testcases Logic
The `test_oracle_ros2.py` uses advanced pattern matching to detect physical and numerical soundness:

| Testcase | Intent & Detection Logic |
| :--- | :--- |
| **Holonomic Kinematics** | Checks for proper **Frame Transformation**. It fails if the AI assumes body-frame velocities ($V_x, V_y$) are already in the global frame. It requires a rotation matrix implementation. |
| **Sonar Geometry** | Validates the **Analytical Derivation** of intersections. It ensures the AI uses the $Dist = \Delta Pos / \text{Direction}$ relationship instead of naive circular approximations. |
| **Numerical Stability** | Detects **Singularity Handling**. It fails if the AI only checks the ray's direction ($dx < 0$). It requires an **Epsilon Guard** (e.g., `abs(dx) > 1e-6`) to prevent division-by-zero for axial rays. |
| **Sensor Range Limit** | Validates **Physical Realism**. It fails if the sonar returns `infinity` (numeric_limits::max) when no wall is hit. It expects a finite `MAX_RANGE` cap. |
| **Sonar Y-Mirroring** | Checks for **Coordinate Alignment**. It verifies that the vertical ray component uses $-sin(\theta)$ to account for Qt's top-down Y-axis drawing convention. |
| **Frame Transformation** | Verifies the output mapping for ROS 2 messages ($y_{pose} = Height - y_{internal}$). |

## 4. Expected Outcome
A successful implementation must demonstrate:
1.  **Kinematic Correctness**: The turtle moves according to its orientation even in holonomic mode.
2.  **Perceptual Accuracy**: The sonar correctly identifies the "First Echo" (shortest distance) among sampled rays in the FOV.
3.  **Numerical Robustness**: The system remains stable (no crashes or `NaN` values) when the turtle travels or looks perfectly parallel to any wall.
4.  **Context Integration**: Proper handling of the `turtlesim` specific Y-axis flip between internal logic and message publication.
