# Task 006: ROS 2 System-Level Integration (Manager-BT-Controller)

## 1. Brief Description
This task simulates a complete "Perception-Decision-Execution" pipeline for an industrial mobile robot. The system consists of three tightly coupled ROS 2 nodes that must work in harmony to perform an autonomous "Search and Pickup" task:

* **Manager (`manage_objects_node.py`)**: Acts as the world state authority. It handles Gazebo entity spawning and manages object states via Service Servers.
* **Behavior Tree (`pickup_behaviors_node.py`)**: The decision-making layer using `py_trees`. It coordinates high-level logic by calling Manager services and updating a shared Blackboard.
* **Controller (`turtlebot_controller_node.py`)**: The actuator layer. It subscribes to Odometry and Goal topics to drive the robot using a Proportional (P) controller.

**The System Challenge**: Moving from ROS 1 to ROS 2 requires more than syntax changes. You must resolve **Service-Client synchronization**, prevent **Executor deadlocks**, and adhere to **ROS 2 Namespace standards** to ensure the entire system doesn't hang.

---
source code directory:
```https://github.com/narcispr/pick_up_objects_task/blob/main/src```


## 2. Hollowing Logic (TODO Objectives)

### A. Manager: Async Spawning Logic
* **Target**: The `spawn_model` function in `manage_objects_node.py`.
* **Logic**: The model must replace the legacy ROS 1 `SpawnModel` (SDF/URDF specific) with the ROS 2 `SpawnEntity` service. It must implement an asynchronous call pattern to allow the node's executor to continue spinning other callbacks (like Odometry).

### B. Behavior Tree: The Async-Sync Bridge
* **Target**: The `update` method in the `CheckObject` class.
* **Logic**: `py_trees` ticks are synchronous, but ROS 2 services are inherently asynchronous. The model must implement a mechanism (like `spin_until_future_complete` or checking `future.done()`) to fetch service results without breaking the BT's tick-tock cycle.

### C. Controller: Node-Centric Communication
* **Target**: The `__init__` constructor in `turtlebot_controller_node.py`.
* **Logic**: The model must migrate from global `rospy` calls to the ROS 2 `Node` object pattern. Crucially, it must strip leading slashes from topic names to support proper ROS 2 remapping and namespaces.

---

## 3. Test Cases & Expected Outcomes

### TC 1: Service Interface Alignment (`test_service_interface_alignment`)
* **Concept**: Ensures the Server (Manager) and Client (BT) are using the exact same service string names.
* **Expected Outcome**: Both files must refer to `check_object` consistently. A mismatch (e.g., one using a namespace and the other not) results in a system-level communication failure.

### TC 2: Global Namespace Compliance (`test_no_leading_slashes`)
* **Concept**: Validates that no topics or services start with `/`. 
* **Expected Outcome**: Topics like `/odom` must be changed to `odom`. This allows the system to be launched in different namespaces (e.g., `robot_1/odom`) without hardcoded global path interference.

### TC 3: Deadlock Prevention (`test_no_nested_spin`)
* **Concept**: Detects if `rclpy.spin` or `spin_until_future_complete` is called inside a member function/callback.
* **Expected Outcome**: **No matches found.** In a single-threaded ROS 2 executor, nesting a spin call inside a callback leads to an immediate system freeze (deadlock).

### TC 4: Communication Robustness (`test_qos_usage`)
* **Concept**: Checks for explicit Quality of Service (QoS) depth.
* **Expected Outcome**: The code must specify a history depth (e.g., `10`) or a `QoSProfile`. ROS 2 requires explicit QoS to ensure reliable data delivery across different network conditions.

### TC 5: Async Logic Integrity (`test_bt_future_handling`)
* **Concept**: Verifies that the Behavior Tree actually waits for and processes the service `Future`.
* **Expected Outcome**: The code must contain logic to check `future.result()` or `future.done()`. Simply sending a request and returning `SUCCESS` without a response check is a logical failure.

---

## 4. System-Level Success Criteria
A successful implementation is defined by the **Inter-node Contract**:
1.  **Handshake**: The BT successfully connects to the Manager's `spawn_entity` service.
2.  **Liveness**: The Manager does not deadlock while waiting for Gazebo, allowing the Controller to receive Odom data simultaneously.
3.  **Portability**: The system functions correctly even if pushed into a ROS 2 Group/Namespace (verified by the lack of leading slashes).
