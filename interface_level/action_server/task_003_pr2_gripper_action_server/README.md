# Task 003: PR2 JointTrajectory Action Server

## Task Description

This task involves translating the ROS1 PR2 JointTrajectory action server into ROS2 C++ code.  
The server receives joint trajectory goals, executes them via the controller, and provides feedback on the joint positions and effort.

**Source C++ (ROS1) highlights:**

- `goalCB(GoalHandle gh)`: handles new goals, checks joint names, cancels previous goals, and publishes trajectories.
- `controllerStateCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)`: monitors controller feedback, verifies constraints, and sets goal success or failure.

**Translation Goal:**  
Ensure the ROS2 version retains the key action semantics while removing ROS1 APIs.

---

## Reason for Blank-Out

For this task, we deliberately **blanked out two core functions** in the source code to guide the LLM translation:

1. `goalCB(GoalHandle gh)`  
2. `controllerStateCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)`

**Reasoning:**  
- These functions contain the key control logic for goal handling and feedback processing.  
- By blanking them, the LLM must **reconstruct the semantic behavior** in ROS2 C++ instead of copying ROS1-specific API calls.  
- This allows us to evaluate whether the translation preserves critical functionality.

---

## Oracle Tests

The Oracle tests are Python `pytest` scripts that **check semantic concepts** in the generated ROS2 C++ code using pattern matching.  
These tests do **not compile or execute the code**, and they run very fast (<1s).

| Test | Concept | Expected Outcome | Notes |
|------|---------|-----------------|------|
| `test_no_ros1_artifacts` | ROS1 APIs removed | Code should not contain `ros::init`, `ros::NodeHandle`, or `actionlib/server/action_server.h` | Ensures translation is ROS2 compliant |
| `test_ros2_action_server_used` | ROS2 ActionServer used | Code should reference `rclcpp_action` or `rclcpp::Node` | Confirms ROS2 action server is instantiated |
| `test_action_type_present` | Action type preserved | Code should reference `JointTrajectoryAction` | Fails if translation changes or drops the original action type |
| `test_node_creation` | Node is created | Code should reference `rclcpp::Node` | Ensures a ROS2 node is correctly instantiated |
| `test_action_server_creation` | Action server is created | Code should instantiate `rclcpp_action::Server` or `ActionServer` | Checks that action server setup exists |
| `test_goalCB_exists` | `goalCB` function exists | Code should define a function matching `goalCB` | Validates that goal handling logic is present |
| `test_goalCB_handles_trajectory` | `goalCB` functionality | Code should reference goal acceptance, rejection, cancel, and trajectory publishing | Ensures semantic behavior of goal handling is preserved |
| `test_controllerStateCB_exists` | `controllerStateCB` function exists | Code should define a function matching `controllerStateCB` | Validates that feedback processing logic exists |
| `test_controllerStateCB_feedback_handling` | Feedback handling | Code should reference `set_succeeded`, `set_aborted`, and trajectory feedback | Ensures semantic behavior of controller feedback is preserved |

---

## Usage

1. Place the translated ROS2 C++ code under `src/task_003/translated/`.  
2. Place this test file under `src/task_003/test/test_oracle_ros2.py`.  
3. Run: `python3 -m pytest src/task_003/test/test_oracle_ros2.py`

