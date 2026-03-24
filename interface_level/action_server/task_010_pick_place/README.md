# Task 010: ROS1 Pick-Place Pipeline Translation to ROS2

## Brief Description
This task evaluates the ability of a Large Language Model (LLM) to translate a ROS1 pick-and-place pipeline into ROS2 code while preserving key semantics. The original ROS1 code uses `MoveGroupInterface` and `PlanningSceneInterface` for grasping and placing a cube with a Panda robot, including gripper control (`openGripper`, `closedGripper`), collision objects, and synchronous pick/place operations. The goal is to generate ROS2 code that:

- Initializes a ROS2 node and spinner
- Creates a `MoveGroupInterface` for the `panda_arm`
- Implements gripper operations correctly at pre-grasp, grasp, and post-place stages
- Preserves pick-and-place logic including collision object manipulation
- Maintains semantic alignment with ROS1

The LLM should fill in the "blanks" of this pipeline while translating ROS1 constructs into their ROS2 equivalents.
## Source code file:
`https://github.com/moveit/moveit_tutorials/blob/master/doc/pick_place/src/pick_place_tutorial.cpp`

We use the same github respository as task 012, but completely different tutorial file.

---

## Blanks and Reasoning

### 1. Node and Spinner Initialization
- **Blank location**: ROS1 `ros::init` and spinner logic
- **Reasoning**: Node initialization and asynchronous spinning are essential for ROS2 operations. This represents a logical unit that must be correctly translated.
- **Todo**: Replace with `rclcpp::init(argc, argv)` and an asynchronous spinner (`rclcpp::AsyncSpinner` or `MultiThreadedExecutor`) that preserves concurrency semantics from ROS1.

### 2. MoveGroupInterface Creation
- **Blank location**: ROS1 `MoveGroupInterface group("panda_arm")`
- **Reasoning**: Planning group initialization is critical for motion planning. This forms a semantic block tied to the robot arm's capabilities.
- **Todo**: Instantiate the ROS2 `MoveGroupInterface` correctly for the same planning group, ensuring planning methods are accessible.

### 3. Gripper Operations
- **Blank location**: `openGripper(pre_grasp_posture)`, `closedGripper(grasp_posture)`, `openGripper(post_place_posture)`
- **Reasoning**: Pre-grasp, grasp, and post-place gripper control forms a logical sequence. Translating ROS1 function calls into ROS2 equivalents ensures the robot manipulates objects correctly.
- **Todo**: Fill in the ROS2 functions to set finger joint trajectories at each stage, maintaining original pre/post grasp logic.

### 4. Pick and Place Logic
- **Blank location**: `pick(group)` and `place(group)`
- **Reasoning**: The pick and place pipelines are the main functional loop. This includes grasp pose setup, approach/retreat vectors, and postures.
- **Todo**: Implement ROS2 versions of pick/place methods that preserve the same grasp logic, support surfaces, and movement semantics.

### 5. Collision Objects
- **Blank location**: `addCollisionObjects(planning_scene_interface)`
- **Reasoning**: Environment modeling via collision objects is crucial for safe motion planning. Preserves the ROS1 semantic of adding tables and cubes.
- **Todo**: Apply collision objects using ROS2 `PlanningSceneInterface`, ensuring objects have the same poses and dimensions as in ROS1.

---

## Oracle Test Cases

The task uses **6 independent pytest Oracle tests**, using regex-based semantic validation without compilation or runtime execution.

### 1. ROS2 Node and Spinner
- **Purpose**: Ensure ROS2 node is initialized and spinner is active for asynchronous execution.
- **Expected Outcome**: Regex matches ROS2 `rclcpp::init` followed by spinner creation.
- **Semantic Correspondence**: ROS1 `ros::init(argc, argv); ros::AsyncSpinner spinner(1); spinner.start();`

### 2. MoveGroupInterface Created
- **Purpose**: Verify a `MoveGroupInterface` exists for `panda_arm`.
- **Expected Outcome**: Regex matches a `MoveGroupInterface` object creation for `panda_arm`.
- **Semantic Correspondence**: ROS1 `MoveGroupInterface group("panda_arm");`

### 3. Pre-Grasp Gripper Open
- **Purpose**: Confirm `openGripper` is called for the pre-grasp posture.
- **Expected Outcome**: Regex matches `openGripper(pre_grasp_posture)` call.
- **Semantic Correspondence**: ROS1 `openGripper(grasps[0].pre_grasp_posture);`

### 4. Grasp Posture Closed
- **Purpose**: Ensure gripper is closed during grasp.
- **Expected Outcome**: Regex matches `closedGripper(grasp_posture)` call.
- **Semantic Correspondence**: ROS1 `closedGripper(grasps[0].grasp_posture);`

### 5. Post-Place Gripper Open
- **Purpose**: Verify gripper is opened after placing an object.
- **Expected Outcome**: Regex matches `openGripper(post_place_posture)` call after `place()` call.
- **Semantic Correspondence**: ROS1 `openGripper(place_location[0].post_place_posture);`

### 6. Pick/Place Pipeline Semantic Integrity
- **Purpose**: Confirm that pick and place pipelines exist as a logical closure.
- **Expected Outcome**: Regex matches pick and place function calls and supports collision object addition.
- **Semantic Correspondence**: ROS1 `addCollisionObjects(planning_scene_interface); pick(group); place(group);`

---

## Notes
- All tests are **semantic**, not syntactic; they check for presence of key constructs, not exact line-by-line matching.
- Each test is independent and ensures the LLM-generated ROS2 code preserves the original ROS1 pick-and-place behavior.
- Tests run in <1 second without compiling the code.
