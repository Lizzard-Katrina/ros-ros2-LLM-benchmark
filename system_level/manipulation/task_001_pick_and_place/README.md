# Task 001: System-Level Pick and Place Migration (ROS 1 to ROS 2)

## 1. Brief Description
This task involves migrating a dual-component robotic manipulation system (Perception + State Machine) from ROS 1 to ROS 2 Humble. The system detects colored blocks using a simulated Kinect sensor in Gazebo and executes a pick-and-place routine. 

Unlike simple script migrations, this is a **System-Level Benchmark**. The core challenge is refactoring the **asynchronous handshake** between the `ObjectDetector` (Perception) and the `PickAndPlaceStateMachine` (Decision) to prevent deadlocks and ensure thread-safe coordination using ROS 2 Executors.

---
source code file:
1. ```https://github.com/elena-ecn/pick-and-place/blob/main/pick_and_place/scripts/pick_and_place_state_machine.py```
2. ```https://github.com/elena-ecn/pick-and-place/blob/main/pick_and_place/scripts/object_detector.py```


## 2. Hollowing Logic & Design Intent

We have hollowed out four critical "linkage points" where the system's synchronization logic resides:

### **Hole 1: Interface & Parameter Initialization (`object_detector.py`)**
* **Reason:** ROS 2 requires explicit parameter declaration and a different node inheritance model. 
* **Design Intent:** Force the model to move away from `rospy.get_param` and implement `self.declare_parameter`. It tests if the model understands the ROS 2 Node lifecycle.

### **Hole 3: Asynchronous Service Retrieval (`object_detector.py`)**
* **Reason:** In ROS 1, `ServiceProxy` is blocking. In ROS 2, calling a service inside a callback using blocking calls causes **Executor Starvation**.
* **Design Intent:** The model must implement `call_async`. We check if it avoids `spin_until_future_complete` inside class methods, which is a common source of nested-spin deadlocks.

### **Hole 3: Async State Transitions (`pick_and_place_state_machine.py`)**
* **Reason:** The State Machine must wait for the `Controller` to finish its move before transitioning to the next state.
* **Design Intent:** This tests the "Handshake." The model must use `Future` callbacks or polling to ensure the state machine doesn't "fire and forget."

### **Hole 4: System Orchestration (`pick_and_place_state_machine.py`)**
* **Reason:** ROS 2 uses **Executors** to manage multiple nodes in a single process.
* **Design Intent:** The model must instantiate a `MultiThreadedExecutor` and add both the Controller and StateMachine nodes to it. This is the "Gold Standard" for system-level correctness.

---

## 3. Oracle Test Design & Expected Outcomes

The Oracle suite uses regex-based static analysis and architectural pattern matching to verify the migration.

| Oracle Test | Design Strategy | Expected Outcome (To Pass) |
| :--- | :--- | :--- |
| `test_detector_async_non_blocking` | Checks for `call_async` and forbids ROS 1 `ServiceProxy`. | The code must use `.call_async(` and contain **zero** instances of `wait_for_service` (blocking). |
| `test_lifecycle_orchestration` | Verifies the use of ROS 2 Executors for multi-node management. | Must contain `MultiThreadedExecutor` (or similar) and use `executor.spin()` instead of `rclpy.spin(node)`. |
| `test_future_synchronization` | Checks if the State Machine actually listens to the Controller's feedback. | Must implement `.add_done_callback(` or `.done()` to guard state transitions. |
| `test_interface_linkage` | Ensures naming consistency for Topics and Message Types across files. | Both files must import from `pick_and_place.msg` and use the same topic string (e.g., `/detected_objects`). |
| `test_parameter_compliance` | Verifies the modern ROS 2 configuration pattern. | Must use `self.declare_parameter(...)` for all external configurations. |
| `test_anti_leakage` | Scans for any residual `rospy` or ROS 1 artifacts. | **Zero** occurrences of `rospy.`, `roslib.`, or ROS 1 specific keywords like `queue_size`. |

---

## 4. Execution Requirement
To pass this benchmark, the migrated code must not only compile but also demonstrate **Architectural Correctness**. A simple API swap that retains ROS 1's blocking logic will fail the **Deadlock Prevention** and **Executor Orchestration** tests.
