# Task 006: ROS1 → ROS2 Action Server Translation (Fill-in-the-Blank)

## Brief Description
This task requires translating a ROS1 action server implementation to ROS2. Instead of providing a complete ROS2 implementation, a **fill-in-the-blank version** of the code is given. Students are expected to implement the missing parts according to ROS2 semantics while preserving the logic and callbacks from the original ROS1 code.

The task tests understanding of:
- ROS2 `rclcpp_action` API
- Action server callbacks: `handle_goal`, `handle_cancel`, `handle_accepted`
- Asynchronous action handling (via `std::thread`)
- Action feedback/result mechanisms

The purpose is **not to compile/run the code** but to ensure semantic equivalence using **oracle tests**.


##Referenced source code:

- github code file: 'https://github.com/ros-planning/navigation/blob/noetic-devel/amcl/src/amcl_node.cpp`

---

## Fill-in-the-Blank (挖空) Design
The ROS2 code provided contains placeholders where key implementations are missing:

1. **Action Server Creation** – students need to instantiate the ROS2 action server.
2. **Goal Handling Callback (`handle_goal`)** – determine whether to accept/reject goals.
3. **Cancel Handling Callback (`handle_cancel`)** – define logic for canceling goals.
4. **Accepted Goal Callback (`handle_accepted`)** – schedule execution, typically asynchronously.
5. **Feedback and Result Handling** – provide feedback updates or mark goals as succeeded.
6. **TODO Comments** – indicate where implementation is required (e.g., particle filter update).

The blanking strategy ensures students focus on **semantic understanding** of ROS2 action server workflow rather than syntax memorization.

---

## Oracle Testcases

All tests are **static checks** using regex/string search on the code, without compilation or execution.

| Test Name | Concept | Design Idea | Expected Outcome to Pass |
|-----------|--------|-------------|-------------------------|
| `test_action_server_creation` | Action server instantiation | Look for `rclcpp_action::create_server<UpdatePose>` pattern | Regex match found → action server is defined |
| `test_handle_goal_defined` | Goal callback | Look for `handle_goal(` pattern | Regex match found → `handle_goal` callback implemented |
| `test_handle_cancel_defined` | Cancel callback | Look for `handle_cancel(` pattern | Regex match found → `handle_cancel` callback implemented |
| `test_handle_accepted_defined` | Accepted goal callback | Look for `handle_accepted(` pattern | Regex match found → `handle_accepted` callback implemented |
| `test_todo_comment_present` | Implementation hint | Look for `// TODO: Implement particle filter update` | Regex match found → placeholder/TODO present for required logic |
| `test_thread_usage_for_async` | Async execution | Look for `std::thread(` usage inside `handle_accepted` | Regex match found → asynchronous handling implemented |
| `test_feedback_or_result_mentioned` | Feedback/result handling | Look for `Feedback`, `Result`, or `goal_handle->succeed` | Regex match found → feedback/result logic referenced |

**Note:** If any regex fails, it indicates the corresponding semantic element is missing in the ROS2 fill-in-the-blank code.

---
