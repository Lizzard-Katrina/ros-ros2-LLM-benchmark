# Task 001 — Robot Calibration Action

## Source Files Used

The task is constructed from the official tutor




## Why these blanks?
The blanks are chosen to target common LLM failure modes:
- forgetting `is_preempt_requested` and `set_preempted`
- mismatching field names vs .action
- incorrect SimpleActionServer/SimpleActionClient usage
- incorrect wait_for_result/cancel behavior

## Oracle Test Cases and Expected Outcomes

### 1. No ROS1 Artifacts

**Checks:**
- No usage of `rospy`
- No `actionlib.SimpleActionServer`
- No ROS1 sleep or node APIs

**Expected Outcome:**
- Test passes if the translation fully removes ROS1 dependencies

---

### 2. ROS2 Action Library Usage

**Checks:**
- Presence of `rclcpp_action`
- Action server creation via ROS2 mechanisms

**Expected Outcome:**
- Test passes if ROS2 action infrastructure is used

---

### 3. Action Type Preservation

**Checks:**
- `RobotCalibration` action type appears in code

**Expected Outcome:**
- Action interface is preserved

---

### 4. Node Creation

**Checks:**
- ROS2 node instantiation is detected

**Expected Outcome:**
- Action server is properly hosted in a node

---

### 5. Execute Callback Logic

**Checks:**
- Presence of execute / goal handling logic

**Expected Outcome:**
- Long-running action semantics preserved

---

### 6. Feedback and Result Messages

**Checks:**
- Feedback and Result message types are referenced

**Expected Outcome:**
- Client-visible progress and outcome preserved

---

### 7. Progress Loop

**Checks:**
- Loop construct (`for` or `while`) exists

**Expected Outcome:**
- Action executes incrementally rather than instantaneously

---

### 8. Action Completion Signaling

**Checks:**
- Result is marked succeeded, aborted, or completed

**Expected Outcome:**
- Action lifecycle is properly closed

---

## Passing Criteria

All oracle tests must pass for the translation to be considered correct at the
interface level.

Partial or incomplete translations will fail one or more tests, providing
actionable feedback on missing semantic components.


## Note:
This is a baseline task, the passing rate should be around 70%
