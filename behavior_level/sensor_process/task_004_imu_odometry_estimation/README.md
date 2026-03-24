# Task 004: UKF State Estimation - 3D Kinematic Projection

## 1. Brief Description
Task 004 focus on the **State Level** of a robotic system. You are required to implement the core "Transition Function" (`projectSigmaPoint`) for an **Unscented Kalman Filter (UKF)** within the industry-standard `robot_localization` framework. 

This function evolves the robot's state (Position, Orientation, Velocity, Acceleration, and Angular Rates) over a discrete time step $\Delta t$, ensuring that the 3D physics remains consistent across all 15 degrees of freedom.

---
source code file:
```https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/src/ukf.cpp```

## 2. Excavation Strategy: The SE(3) Challenge
The excavation targets the internal logic of the motion model. Unlike simple 2D unicycle models, this task is "hollowed out" to test the model's internal understanding of **3D Manifold Geometry**:
* **Rotational Coupling**: Linear velocities are defined in the body frame but must be integrated in the global frame. This requires a full $R_{zyx}$ rotation matrix.
* **Angular Kinematics**: Mapping body-fixed angular rates to Euler angle derivatives is non-linear and involves trigonometric singularities (Gimbal Lock proximity).
* **Higher-Order Integration**: Position updates must account for constant acceleration ($1/2 a t^2$) to maintain filter stability during high-dynamic maneuvers.

## 3. Oracle Test Design & Design Intent

The Oracle suite is designed to detect "shallow" implementations that use simplified 2D physics or ignore framework-specific optimizations.

| Test Case | Design Intent | Expected Outcome (To Pass) |
| :--- | :--- | :--- |
| **3D Rotation Coupling** | **Physical Fidelity.** Ensures the model realizes that $X$ displacement depends on both Pitch and Yaw. | Inclusion of the composite term `cos(yaw) * cos(pitch)` in the $X$ update logic. |
| **Acceleration 3D Projection** | **Dynamic Consistency.** Checks if acceleration is correctly rotated into the global frame. | Implementation of $0.5 \cdot R(\theta) \cdot a \cdot \Delta t^2$ where $R$ is the 3D rotation matrix. |
| **Singularity Mapping** | **Mathematical Robustness.** Validates the non-linear mapping of angular rates. | Use of `1/cos(pitch)` (`cpi`) or `tan(pitch)` (`tp`) to evolve Roll and Yaw. |
| **Index Safety Enforcement** | **Architecture Compliance.** Prevents hard-coded magic numbers which break maintainability. | **Zero** use of integer indices (e.g., `(0,3)`); mandatory use of `StateMember` enums. |
| **Eigen Optimization** | **Performance Standard.** Ensures high-frequency real-time compatibility. | Use of `applyOnTheLeft()` to update the sigma point in-place without temporary copies. |

## 4. Expected Behavioral Outcome
A successful implementation will allow the UKF to track a robot accurately in **full 3D space** (e.g., a drone or a ground robot on uneven terrain). 

**Failure Modes identified by Oracle:**
* **"The 2D Trap"**: Implementing $x = x + v \cdot dt$ without 3D rotation (Fails `test_3d_rotation_coupling`).
* **"The Linear Bias"**: Assuming angular rates map 1:1 to Euler angles (Fails `test_angular_singularity_mapping`).
* **"The Scripting Habit"**: Using hard-coded indices like `matrix(0,3)` instead of framework enums (Fails `test_index_safety`).

## 5. Metadata
* **Level**: State Level (Estimation)
* **Complexity**: High (Mathematical/Kinematic)
* **Framework**: ROS 2 / robot_localization / Eigen
