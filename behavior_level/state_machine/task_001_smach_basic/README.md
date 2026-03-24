# Benchmark Task: SMACH ServiceState Migration (ROS to ROS 2)

## 1. Brief Description
This task involves migrating the `ServiceState` class from the `smach_ros` package. `ServiceState` is a functional state in the SMACH state machine that acts as a ROS Service Client. 

The migration requires moving from the synchronous, global-node-based architecture of **ROS 1 (Noetic)** to the node-handle and callback-group-based asynchronous architecture of **ROS 2 (Humble/Jazzy)**. The core challenge is implementing a synchronous "blocking" behavior within the `execute` loop without deadlocking the ROS 2 executor.

--
source code file:
```https://github.com/ros/executive_smach/blob/ros2/smach_ros/smach_ros/service_state.py```

## 2. Hollowing Strategy (Hole-filling)
The task targets the **Behavioral Logic** of the state machine. We hollow out the mid-section of the `execute(self, ud)` method.

- **Start Point:** The beginning of the service readiness check (`while not self._proxy.service_is_ready():`).
- **End Point:** Just before the response outcome processing (`if self._response_key is not None:`).

### Why this section?
- **Communication Paradigm Shift:** It forces the LLM to replace `rospy.ServiceProxy` logic with `rclpy` client calls.
- **Asynchronous Handling:** ROS 2 services are inherently asynchronous. The LLM must implement a `while/spin_once` or `spin_until_future_complete` pattern to maintain SMACH's synchronous state execution.
- **Preemption Integrity:** The LLM must ensure that the state remains "responsive" to SMACH preemption signals even while waiting for a ROS service response.

## 3. Oracle Test Design & Expected Outcomes

The oracle tests use **Pattern Matching (Regex)** to verify semantic correctness without requiring a running ROS 2 environment.

| Test Case | Design Logic | Expected Outcome (Success Pattern) |
| :--- | :--- | :--- |
| `test_ros2_client_init` | Checks if the client is created via a Node handle. | `self.node.create_client(...)` |
| `test_service_ready` | Verifies the use of ROS 2 service discovery APIs. | `self._proxy.wait_for_service(...)` or `service_is_ready()` |
| `test_preemption` | Ensures the state checks for `preempt_requested()` inside loops. | `while ...: if self.preempt_requested(): ...` |
| `test_ros2_logging` | Ensures `rospy.log*` is replaced by the Node logger. | `self.node.get_logger().<level>(...)` |
| `test_service_call` | Checks for the execution of the service request. | `self._proxy.call_async(...)` or `self._proxy.call(...)` |
| `test_spin_logic` | Verifies that the LLM drives the executor to avoid deadlocks. | Presence of `rclpy.spin_once` or equivalent if using async. |
| `test_userdata_mapping` | Verifies that input/output keys are mapped to the request/response. | `setattr(self._request, ..., ud[...])` |
| `test_no_legacy` | Negative test to ensure no ROS 1 artifacts remain. | Absence of `rospy`, `ServiceProxy`, or `init_node`. |

### Expected Outcome
A successful migration should pass **all 8 tests**. 
- **Note on Variability:** The tests are designed to be variable-name agnostic (e.g., accepting `key` or `slot`) as long as the functional mapping logic `ud -> request` and `response -> ud` is preserved.
- **Performance:** All oracle tests should execute in under **0.2 seconds** via `pytest`.
