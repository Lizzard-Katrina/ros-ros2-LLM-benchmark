# Task 009 — MoveIt Motion Planning Action


## Brief Description
This task evaluates the translation of a ROS1 MoveGroup action server to ROS2.  
Specifically, the LLM is expected to translate the `MoveGroupMoveAction` capability from ROS1 `actionlib` to ROS2 `rclcpp_action`, preserving semantic behavior, including:

- Action server initialization and callback registration  
- Handling `plan_only` and `plan_and_execute` requests  
- Preemption and cancellation logic  
- Publishing move feedback  
- Avoiding ROS1 remnants (e.g., `ros::NodeHandle`, `ROS_INFO`)

The benchmark uses semantic-oriented oracle testcases to ensure that the translated code retains the critical logic of the original ROS1 implementation while following ROS2 syntax and patterns.

---

## Source code file:

`https://github.com/moveit/moveit/blob/master/moveit_ros/move_group/src/default_capabilities/move_action_capability.cpp`


---

## 1. Blank-Out Strategy
### Purpose
The "blanking" aims to remove core logic from `executeMoveCallback` to create a meaningful benchmark:

- **Why:**  
  - The action server is the central loop of motion planning.  
  - Removing it prevents trivial copying solutions and forces the LLM to reconstruct correct ROS2 semantics.  

- **Scope of blanking:**  
  - Entire body of `executeMoveCallback` (both `plan_only` and `plan_and_execute` branches)  
  - Preempt handling logic inside `preemptMoveCallback`  
  - Feedback publishing inside `setMoveState`

- **Result:**  
  The LLM must fill in these sections to pass semantic-level tests, preserving correct planning, execution, preemption, and feedback behavior.

---

## 2. Oracle Testcase Design

### Test 1: `test_class_exists`
- **Purpose:** Ensure the core capability class is present.  
- **Expected Outcome in Code:** `class MoveGroupMoveAction` is defined.  
- **Reference ROS1 code:**  
```cpp
class MoveGroupMoveAction : public MoveGroupCapability { ... };
```
- **Reasoning**: If missing, the LLM likely did not correctly translate the ROS1 capability structure.
### ✅ Testcase 2: `test_action_server_usage`

**Purpose:** Verify that an ROS2 action server is instantiated for the `MoveGroup` action.  

**Expected Outcome in Code:**  
- `rclcpp_action::Server<moveit_msgs::action::MoveGroup>`  
- or `create_server<moveit_msgs::action::MoveGroup>`  
**Reference ROS1 Code:**  
```cpp
move_action_server_ = std::make_unique<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>>( ... );
```

### ✅ Testcase 3: `test_rclcpp_node_exists`

**Purpose:** Ensure a ROS2 node exists in the translated code.  

**Expected Outcome in Code:**  
- `std::make_shared<rclcpp::Node>`  
- or usage of `rclcpp::Node::SharedPtr`  
**Reference ROS1 Code:**  
```cpp
ros::NodeHandle node_handle_;
```

---

### ✅ Testcase 4: `test_initialize_creates_server_with_callback`

**Purpose:** Ensure `initialize()` exists and correctly registers the action server callback.  

**Expected Outcome in Code:**  
- `void MoveGroupMoveAction::initialize()` exists  
- The server creation binds to `executeMoveCallback`  

**Reference ROS1 Code:**  
```cpp
move_action_server_ = std::make_unique<actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>>(
    root_node_handle_, MOVE_ACTION, boost::bind(&MoveGroupMoveAction::executeMoveCallback, this, _1), false);
```


---

### ✅ Testcase 5: `test_execute_callback_sets_result`

**Purpose:** Ensure `executeMoveCallback` sets goal results and handles both `plan_only` and `plan_and_execute` logic.  

**Expected Outcome in Code:**  
- Calls like `goal_handle->succeed()` or `goal_handle->abort()`  
- Contains `plan_only` and `plan_and_execute` branching  

**Reference ROS1 Code:**  
```cpp
if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  executeMoveCallbackPlanOnly(goal, action_res);
else
  executeMoveCallbackPlanAndExecute(goal, action_res);
```


---

### ✅ Testcase 6: `test_preempt_callback_handles_cancel_and_flag`

**Purpose:** Verify preemption logic in the ROS2 translation.  

**Expected Outcome in Code:**  
- `preempt_requested_ = true`  
- `context_->plan_execution_->stop()` or equivalent  

**Reference ROS1 Code:**  
```cpp
void MoveGroupMoveAction::preemptMoveCallback() {
  preempt_requested_ = true;
  context_->plan_execution_->stop();
}
```


---

### ✅ Testcase 7: `test_setMoveState_publishes_feedback_with_state`

**Purpose:** Validate that feedback is published to the client whenever the move state changes.  

**Expected Outcome in Code:**  
```cpp
move_feedback_.state = stateToStr(state);
move_action_server_->publishFeedback(move_feedback_);
```


---

### ✅ Testcase 8: `test_no_ros1_artifacts`

**Purpose:** Ensure the translated code does not retain ROS1 functions, headers, or types.  

**Expected Outcome in Code:** No occurrences of:  
- `ros::init`  
- `ros::NodeHandle`  
- `ROS_INFO`  
- `boost::shared_ptr`  

**Reference ROS1 Code:** All original ROS1 initialization and logging lines.

**Design Reasoning:**  
Guarantees that the translation is fully ROS2-compliant and free of legacy ROS1 constructs.


