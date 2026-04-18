# Benchmark Task: TM Robot ROS2 Driver Service Implementation

## 1. Brief Description
This task involves implementing the `tm_msgs/srv/AskItem` service within a ROS2 driver framework for Techman Robots (TM Robot). The service allows a client to query internal controller states (e.g., `HandCamera_Value`, `DeltaDH`) using a custom communication protocol. 

The primary goal of this benchmark is to test the developer's ability to write code that not only functions correctly but also adheres to **strict coding standards and naming conventions** enforced by regex-based static analysis (Oracle Test).

---
source code file directory:
```https://github.com/TechmanRobotInc/tmr_ros1/blob/melodic/tm_demo```

## 2. Design Thinking (Code Hole Design)

### C++: `tm_communication.cpp` (Socket Error Handling)
* **Objective**: Test the handling of TCP connection closures.
* **Design**: In a `select()`-based polling model, `recv()` returning `0` is the standard indicator of a closed connection. The regex enforces a direct check within the `if` statement to ensure concise and idiomatic error handling without redundant intermediate variables.

### C++: `tm_ros_service.cpp` (Thread Synchronization)
* **Objective**: Implement a thread-safe "Request-Response" bridge between the asynchronous ROS2 service and the synchronous robot protocol.
* **Design**: 
    * **Mutex Naming**: Forces the use of `svr_mtx_` to simulate adherence to a legacy or enterprise-wide naming convention.
    * **Notification Loop**: Developers must implement both the `wait_for` logic in the service call and the `notify_all()` logic in the data-receiver callback to prevent deadlocks and service timeouts.

### Python: `ask_item_demo.py` (String Sanitization)
* **Objective**: Standardize the parsing of the robot's specific protocol format (where data is wrapped in `{}`).
* **Design**: The benchmark mandates the use of the `.strip('{}')` method. This tests whether the developer uses the most readable and efficient Python string API rather than manual slicing or regex, which can be error-prone.

---

## 3. Testcase Design & Expected Code Outcomes

| Testcase | Concept / Regex Pattern | Expected Code Outcome |
| :--- | :--- | :--- |
| `test_recv_error_handling` | `recv\(.*?\)\s*==\s*0` | Must use `if (recv(...) == 0)` directly to catch closed sockets. |
| `test_mutex_safety` | `(?:unique_lock\|lock_guard).*?svr_mtx_` | Shared state access must be wrapped in a lock using the specific variable name `svr_mtx_`. |
| `test_svr_callback_notification` | `svr_cond_\.notify_(?:all\|one)\s*\(` | The callback function must explicitly call `notify_all()` to wake the blocked service thread. |
| `test_python_brace_stripping` | `\.strip\s*\(\s*['\"].*?[{}]` | Must use `.strip('{}')` to remove protocol braces from the response content. |
| `test_demo_blocking_call` | `ask_item\(.*?,.*?,[^0]\d*\)` | The demo must implement a blocking call by passing a plain integer (e.g., `5`) to `wait_time`. |

