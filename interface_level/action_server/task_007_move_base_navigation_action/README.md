# Task 007 — Move Base Navigation Action

# Task 007: Move Base Navigation Action (ROS1 → ROS2)

## Task Type

- **Level**: Interface-level
- **Interface Category**: Action Server
- **Domain Context**: Navigation
- **Evaluation Focus**: Action lifecycle semantics, feedback/result handling, concurrency-safe execution

> ⚠️ This task is **NOT system-level**.
> Although the original source code comes from a large navigation stack, this benchmark
> evaluates only the **Action Server interface semantics**, not full navigation behavior,
> multi-node coordination, or runtime correctness.

---

## 1. Hollowed File Location in the Original GitHub Project

**Reference Code github**: ```https://github.com/ros-planning/navigation/blob/noetic-devel/move_base/src/move_base.cpp```
**Original repository**: `ros-planning/navigation`  
**ROS1 package**: `move_base`

**Hollowed source file**:

```text
navigation/
└── move_base/
    └── src/
        ├── move_base.cpp   ← hollowed in this task
        └── move_base_node.cpp
```

### Why move_base.cpp

1. Contains the core Action Server execution loop

2. Implements:
- Action feedback publication
- Goal success / abort signaling
- Long-running execution semantics
- Mutex-protected shared state access

3. Exposes the interface-visible behavior of the navigation action

##Hollowing Strategy

###Hollowed Scope

A single closed-loop execution segment inside the move_base Action Server execution
function is removed.

This segment corresponds to one iteration of action execution and includes:

- Feedback construction and publication

- Planner/controller state synchronization under mutex

- State-machine-driven transitions:

- Goal termination decisions:

     Success

     Abort

- Recovery behavior invocation

## Oracle Tests


Oracle tests validate the translated ROS2 C++ code using **static source analysis only** (regex/string search).  
No compilation or runtime execution is required.  

**Total oracle tests**: **9**  
**Test file location**: `task_007/test/test_oracle_ros2.py`

Each test corresponds to one **interface-level semantic concept** of the Action Server.

---

### Test Case 1: Mutex Translation
**Concept**: ROS1 `boost::recursive_mutex` locks must be translated to ROS2-compatible locks.  
**Expected Outcome**: Presence of `std::unique_lock`, `std::lock_guard`, or ROS2 mutex equivalents; no ROS1-style scoped locks remain.  
**Original ROS1 Code Reference**:
```cpp
boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
```

### Test Case 2: Action Feedback Publication
**Design:** Verify that the action server still publishes feedback to report the robot's position.  
**Expected Outcome:** The ROS2 code should call `as_->publishFeedback`.  
**Original ROS1 Code Location:** `as_->publishFeedback(feedback);`

### Test Case 3: Plan Swap Under Mutex
**Design:** Ensure that swapping pointers of `controller_plan_` and `latest_plan_` occurs under a mutex lock.  
**Expected Outcome:** ROS2 code contains the pointer swap (`controller_plan_ = latest_plan_`, `latest_plan_ = temp_plan`) under a lock (`std::unique_lock` or `rclcpp::Mutex`).  
**Original ROS1 Code Location:**  
```cpp
boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
controller_plan_ = latest_plan_;
latest_plan_ = temp_plan;
lock.unlock();
```


### Test Case 4: Action Server Success Signaling
**Design:** Check that the action server signals successful completion to clients.  
**Expected Outcome:** ROS2 code contains `as_->setSucceeded`.  
**Original ROS1 Code Location:** `as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");`

### Test Case 5: Action Server Abort Signaling
**Design:** Check that the action server signals aborted execution when necessary.  
**Expected Outcome:** ROS2 code contains `as_->setAborted`.  
**Original ROS1 Code Location:** Multiple points, e.g.,  
```cpp
as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
```

### Test Case 6: Navigation State Machine Preservation
**Design:** Ensure the main move_base state machine with `PLANNING`, `CONTROLLING`, and `CLEARING` cases is preserved.  
**Expected Outcome:** ROS2 code contains `case PLANNING`, `case CONTROLLING`, and `case CLEARING`.  
**Original ROS1 Code Location:** `switch(state_) { case PLANNING: ... case CONTROLLING: ... case CLEARING: ... }`
### Test Case 7: Velocity Command Publication
**Design:** Verify that velocity commands are sent to the robot interface.  
**Expected Outcome:** ROS2 code publishes velocity commands via `vel_pub_->publish(cmd_vel)`.  
**Original ROS1 Code Location:** `vel_pub_.publish(cmd_vel);`
### Test Case 8: Safety Checks Before Motion
**Design:** Ensure that the costmap is checked for freshness before commanding the robot, to prevent unsafe motion.  
**Expected Outcome:** ROS2 code checks `!controller_costmap_ros_->isCurrent()` and calls `publishZeroVelocity()` if outdated.  
**Original ROS1 Code Location:**  
```cpp
if(!controller_costmap_ros_->isCurrent()){
  publishZeroVelocity();
  return false;
}
```


### Test Case 9: Recovery Behavior Invocation
**Design:** Verify that recovery behaviors are invoked when local planner cannot produce a valid command.  
**Expected Outcome:** ROS2 code calls `runBehavior()` for recovery behaviors.  
**Original ROS1 Code Location:**  
```cpp
recovery_behaviors_[recovery_index_]->runBehavior();
```
