# Task: ROS 1 to ROS 2 Migration for Reach ROS

## 1. Brief Description
This task involves the manual migration of the `reach_ros` industrial inspection planning framework from **ROS 1 (Noetic)** to **ROS 2 (Humble)**. The migration targets three critical functional areas: **Inverse Kinematics (IK) Solver**, **Robot State Evaluation (Manipulability)**, and **Coordinate Transformations (TF2)**. 

The primary objective is to replace the deprecated `roscpp` architecture with **MoveIt 2** `RobotModel` and `RobotState` APIs while ensuring thread-safe, modern C++17 standards and proper lifecycle management.

---

## 2. Design Philosophy of TODO Blocks (Task Holes)

Each "hole" in the source code is strategically placed to test a specific pillar of the ROS 2 / MoveIt 2 ecosystem:

### A. MoveIt 2 IK Logic (`moveit_ik_solver.cpp`)
* **Design Intent**: To verify the developer's understanding of the `RobotState` lifecycle in MoveIt 2.
* **Design Logic**: Unlike ROS 1, MoveIt 2 requires an explicit `state.update()` call after setting joint positions to refresh the internal transform tree. The hole requires implementing `setFromIK` with a callback (using `boost::bind` or Lambda) to ensure collision-aware IK, which is a standard safety requirement in industrial planning.

### B. Jacobian & Score Calculation (`manipulability_moveit.cpp`)
* **Design Intent**: To test Eigen integration and the ability to extract partial Jacobians for redundant systems.
* **Design Logic**: This hole requires the developer to retrieve the Jacobian matrix for a specific `JointModelGroup` and handle row subsets. It tests whether the developer can correctly map the MoveIt 2 state information into Eigen matrices for singular value decomposition (SVD).

### C. TF2 Buffer & Lookup (`transformed_point_cloud_target_pose_generator.cpp`)
* **Design Intent**: To test the significantly overhauled TF2 API in ROS 2.
* **Design Logic**: ROS 2 TF2 lookups use `tf2::TimePointZero` instead of `ros::Time(0)` and require explicit `std::chrono` or `tf2::duration` for timeouts. The hole also enforces the use of the `tf2_eigen` bridge for seamless conversion from `TransformStamped` to `Eigen::Isometry3d`.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle (`test_oracle_ros2.py`) uses **Regex-based Static Analysis** to ensure both functional correctness and architectural compliance.

### Test 1: `test_ros1_remnants_check`
* **Design Logic**: Scans all migrated files for "ROS 1 pollution." It looks for `ros::` namespaces, `ros/ros.h` headers, and old logging macros like `ROS_INFO`.
* **Expected Outcome**: 
    * **Pass**: All ROS 1 remnants are replaced by `rclcpp::`, `<rclcpp/rclcpp.hpp>`, and `RCLCPP_INFO`.
    * **Fail**: Any leftover ROS 1 syntax is detected, indicating an incomplete migration.

### Test 2: `test_ros2_compliance_and_ik`
* **Design Logic**: Validates the **MoveIt 2 Lifecycle**. It specifically searches for the `state.update()` call and ensures the IK solver is linked to the `isIKSolutionValid` collision callback.
* **Expected Outcome**: 
    * **Pass**: Code calls `update()` and uses either `setFromIK` or `searchPositionIK` with a valid callback reference.
    * **Fail**: If `update()` is missing (leading to stale transforms) or the collision check logic is bypassed.

### Test 3: `test_tf2_ros2_migration`
* **Design Logic**: Checks for modern TF2 signatures, specifically looking for `tf2::TimePointZero`, `tf2::transformToEigen`, and proper `std::chrono` or `tf2::duration` usage.
* **Expected Outcome**: 
    * **Pass**: Correct use of ROS 2 timing handles and successful application of the transform to the result vector.
    * **Fail**: Reversion to `ros::Duration` or failure to apply the transformation logic.

### Test 4: `test_manipulability_eval_logic`
* **Design Logic**: Ensures the mathematical pipeline for evaluation remains intact by checking for Jacobian retrieval and SVD calls.
* **Expected Outcome**: 
    * **Pass**: Successful integration of `getJacobian`, `Eigen::JacobiSVD`, and `singularValues`.
    * **Fail**: Missing SVD decomposition or incorrect Jacobian matrix handling.

### Final Validation
To pass the task, the implementation must achieve:
**`4 passed in X.XXs`** when running the command `python3 -m pytest src/test/test_oracle_ros2.py`.
