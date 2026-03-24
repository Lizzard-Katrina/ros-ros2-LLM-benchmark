# Task 002: UR5 Pick-and-Place — Manipulation Integration

## Level
Integration-Level Correctness — Manipulation

## Task Goal

This task evaluates whether a robotic manipulation pipeline is **correctly integrated**
from motion planning to execution orchestration.

The benchmark focuses on **system-level correctness**, not low-level physics
or perception accuracy.

---

---

## TODO Files and Source Mapping

### 1. motion_planning_todo.py

- Source directory:
motion_planning/scripts/motion_planning.py


- Responsibility:
Generate a valid joint-space trajectory for the UR5 robot
given a target end-effector pose.

- Expected Outcome:
- Returns a non-empty trajectory
- Trajectory controls all UR5 joints
- Trajectory is kinematically feasible

---

### 2. controller_todo.py

- Source directory:
controller.py

- Responsibility:
Orchestrate a complete pick-and-place routine using the motion planner.

- Expected Outcome:
- Calls motion planner for each manipulation stage
- Executes steps in correct semantic order
- Stops execution if any planning step fails

---

## Notes

- Grasp attachment and Gazebo physics are **out of scope**
- This benchmark does **not** evaluate trajectory optimality
- Correct integration logic is the primary evaluation target

# Motion Planning & Controller Oracle Tests (ROS2)

This directory contains **oracle tests** for the translation of the original ROS1 motion planning and controller code to ROS2.  
The purpose is to verify **system-level correctness** of the robotic manipulation pipeline without running the full simulation.

## Tested Components

| Todo File | Original Source Path | Oracle Test Focus |
|-----------|--------------------|-----------------|
| motion_planning.py | motion_planning/scripts/motion_planning.py | - Node inherits from rclpy.node.Node<br>- MODELS_INFO exists<br>- straighten method exists |
| controller.py | motion_planning/scripts/controller.py | - ROS2 publisher exists<br>- move_to method callable and accepts position + quaternion<br>- gripper_state attribute exists |

## How to Run

1. Build the Docker container (see Dockerfile in this directory).
2. Start the container.
3. Inside the container, run:

```bash
pytest test_motion_planning_oracle.py
pytest test_controller_oracle.py
