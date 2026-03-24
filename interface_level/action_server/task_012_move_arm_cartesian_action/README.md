# Task 012: MoveIt ROS1 → ROS2 Translation

## 1. Brief Description

This task requires translating key logic from a **ROS1 MoveIt tutorial** (`move_group_interface_tutorial.cpp`) to **ROS2**, focusing on motion planning, Cartesian paths, and collision object handling.

**Learning goals:**

- Initialize and spin a ROS2 node asynchronously
- Instantiate and use `MoveGroupInterface` for planning groups
- Plan motions to **pose targets** and **joint-space goals**
- Apply **path constraints** for end-effector orientation
- Compute **Cartesian paths** through waypoints
- Add, attach, detach, and remove **collision objects** in the planning scene

Students will **fill in missing logic blocks** in the ROS2 skeleton code to reproduce the core behaviors.

---

### Source code file: 

`https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp`
---

## 2. Code Blanks (What to Fill In)

The task skeleton contains **strategic blanks** corresponding to major logical blocks from the ROS1 tutorial. Each blank represents a **semantic concept**, not just a single line.

| Blank Location | Concept / Logic | Reason for Blank |
|----------------|----------------|-----------------|
| ROS2 Node Initialization | Initialize ROS2, create node, and start async spinner | Tests whether student correctly sets up a spinning ROS2 node, which is required for MoveGroupInterface to receive robot state updates. |
| MoveGroupInterface Instantiation | Create `MoveGroupInterface` for a planning group | Ensures the student understands how to set up planning for a specific robot arm/group. |
| Path Constraints | Define `OrientationConstraint` and wrap in `Constraints` object | Tests understanding of applying end-effector orientation constraints during planning. |
| Cartesian Path Computation | Create a `std::vector<geometry_msgs::Pose>` waypoints and call `computeCartesianPath()` | Ensures the student knows how to define Cartesian trajectories via waypoints. |
| Collision Object Handling | Define `CollisionObject`, add it to planning scene, optionally apply with `applyCollisionObject()` | Tests understanding of adding, attaching, detaching, and removing objects to/from the environment for safe motion planning. |

> Each blank is intended to encapsulate a **full logical step** from the original ROS1 tutorial, so students must translate **the entire segment**, not just a single function call.

---

## 3. Oracle Testcases


The oracle tests use **regex/string matching** to validate **semantic concepts** in student ROS2 code. Tests are independent and run in <1 second.

| Test Name | Concept Tested | Expected Outcome | ROS1 Reference Segment |
|-----------|----------------|-----------------|-----------------------|
| `test_ros2_node_initialization` | ROS2 node setup with async spinner | Regex matches `rclcpp::init(...)` followed by `rclcpp::AsyncSpinner` | `ros::AsyncSpinner spinner(1); spinner.start();` |
| `test_move_group_interface_exists` | MoveGroupInterface instantiation for PLANNING_GROUP | Regex matches `MoveGroupInterface` instance for `PLANNING_GROUP` | `moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);` |
| `test_pose_target_planning` | Pose target planning | Regex matches `setPoseTarget(...)` and `plan(...)` calls | `move_group_interface.setPoseTarget(target_pose1); move_group_interface.plan(my_plan);` |
| `test_joint_space_planning` | Joint-space planning | Regex matches `setJointValueTarget(...)` and `plan(...)` calls | `move_group_interface.setJointValueTarget(joint_group_positions); move_group_interface.plan(my_plan);` |
| `test_path_constraints` | Path/orientation constraints | Regex matches `OrientationConstraint` and `Constraints` | `moveit_msgs::OrientationConstraint ocm; moveit_msgs::Constraints test_constraints; move_group_interface.setPathConstraints(test_constraints);` |
| `test_cartesian_path_computation` | Cartesian path via waypoints | Regex matches `std::vector<geometry_msgs::Pose>` and `computeCartesianPath()` | `std::vector<geometry_msgs::Pose> waypoints; move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);` |
| `test_collision_object_added` | Collision object creation and addition | Regex matches `CollisionObject`, `addCollisionObjects()`, and optionally `applyCollisionObject()` | `moveit_msgs::CollisionObject collision_object; planning_scene_interface.addCollisionObjects(collision_objects);` |
| `test_object_attached_to_robot` | Attach/detach object to robot | Regex matches `attachObject(...)` and `detachObject(...)` calls | `move_group_interface.attachObject(object_to_attach.id, "panda_hand", {...}); move_group_interface.detachObject(object_to_attach.id);` |
| `test_visual_tools_usage` | MoveItVisualTools instantiation and marker publishing | Regex matches `MoveItVisualTools` object creation and `publish*()` calls | `moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0"); visual_tools.publishText(...); visual_tools.publishTrajectoryLine(...);` |


### Notes on Oracle Tests

- Each test is **independent** and should pass in <1 second.
- Tests match **semantic meaning**, not exact variable names or formatting.
- Failure indicates missing logic/concept translation, not syntax errors.
- Students must fill in the **entire logical block**, not just a function call, to pass all tests.
