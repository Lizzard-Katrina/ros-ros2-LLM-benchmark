# Task 002: Advanced ROS 2 Arm Manipulation and Trajectory Migration

## 1. Brief Description
This task focuses on migrating a complex robotic manipulation sequence from ROS 1 to **ROS 2 Humble**. The scenario involves controlling a UR5-style arm in Gazebo to identify, "straighten" (re-orient), and pick-and-place Lego bricks. 

The core challenge is transitioning from the synchronous `rospy` environment to the asynchronous, executor-based architecture of `rclpy`, while ensuring physical safety through end-effector trajectory interpolation.

---
source code file:
1. ```https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/blob/main/catkin_ws/src/motion_planning/scripts/motion_planning.py```
2. ```https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/blob/main/catkin_ws/src/motion_planning/scripts/controller.py```


## 2. Design Logic for Implementation (Holes)

### Hole 1: Trajectory Interpolation (`controller.py`)
* **Logic:** In a high-fidelity Gazebo simulation, sending a single goal pose causes the physics engine to "teleport" the arm, leading to instability or solver crashes. 
* **Requirement:** The developer must implement a motion loop in `move_to`. It must use **Slerp (Spherical Linear Interpolation)** for orientations and linear interpolation for positions to generate a smooth sequence of `JointTrajectoryPoint` messages.

### Hole 2: Manipulation Orchestration (`motion_planning.py`)
* **Logic:** This tests the ability to coordinate multiple ROS 2 interfaces (Actions and Services) within a logical loop.
* **Requirement:** Implement a sequence that:
    1. Detects the brick's current facing axis.
    2. Executes a "straighten" maneuver if the brick is not upright.
    3. Manages model state using `Attach` services for grasping and `SetStatic` for final placement.

---

## 3. Test Case Design and Expected Outcomes

| Test Case | Design Intent | Expected Outcome (Pass Criteria) |
| :--- | :--- | :--- |
| **test_no_nested_deadlocks** | Prevents the most common ROS 2 migration failure: nested spinning. | **Fail** if `spin_until_future_complete` is used inside a class method. **Pass** if the node handles futures via callbacks or async patterns. |
| **test_trajectory_integrity** | Ensures the robot doesn't "cheat" by removing the motion loop for speed. | **Fail** if the `move_to` logic lacks a `for` loop or the `slerp` method. **Pass** if smooth interpolation is detected. |
| **test_strict_naming** | Enforces the specific naming convention required by the system-level benchmark. | **Fail** if service clients are renamed (e.g., `attach_client`). **Pass** if named exactly `self.attach_srv`, `self.detach_srv`, etc. |
| **test_manipulation_flow** | Verifies the operational integrity of the pick-and-place logic. | **Pass** if the script calls all critical functions: `straighten`, `move_to`, `open_gripper`, and `set_model_fixed`. |
| **test_ros2_interface** | Validates the use of correct ROS 2 messaging standards. | **Pass** if `trajectory_msgs.msg` and `ActionClient` are properly instantiated and used. |
