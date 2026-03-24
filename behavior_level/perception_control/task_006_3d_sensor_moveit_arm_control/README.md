# Task 006: ROS 1 to ROS 2 Pick-and-Place Pipeline Translation

## 1. Brief Description
This task evaluates the capability of a Large Language Model (LLM) to migrate a complex robot manipulation pipeline from **ROS 1 (ActionLib + MoveIt)** to **ROS 2 (rclpy + MoveIt 2)**. 

Beyond simple API replacement, the task focuses on the model's understanding of the **Asynchronous Programming Model (Async/Await)** in ROS 2, Node lifecycle management, and the synchronization of the **Planning Scene**. It serves as a rigorous benchmark for practical robotics engineering.

---
Source code file:
```https://github.com/pal-robotics/tiago_tutorials/blob/noetic-devel/tiago_pick_demo/scripts/pick_and_place_server.py```
## 2. Implementation Holes & Logic
The task includes three critical `TODO` sections where the model must implement core robotic logic:

### A. Planning Scene Polling (`wait_for_planning_scene_object`)
* **Logic**: In ROS 2, planning scene updates are not instantaneous. The model must implement a polling mechanism that repeatedly queries the environment state and inspects the collision object list to ensure the target part has successfully entered the simulation before proceeding.

### B. Grasp Pipeline Construction (`grasp_object`)
* **Logic**: To ensure reliable manipulation, the model must implement a standard pre-processing sequence. This includes clearing legacy collision objects, sequentially adding both the target object and its supporting surface (e.g., a table), and explicitly synchronizing with the scene server before triggering the action goal.

### C. Asynchronous Fallback Mechanism (`place_object`)
* **Logic**: In real-world scenarios, planning with only the arm may fail due to singularities. The model is required to construct a conditional logic flow that detects a failed placement attempt with the primary group and automatically retries the operation using a more versatile group (e.g., arm + torso).

---

## 3. Oracle Testcase Design & Expected Outcomes
The Oracle Tests use a "Tight Logic" approach, combining multiple sub-steps into atomic semantic checks to ensure high-quality code generation.

### Test 1: ROS 2 Environment Purity (Base Structure)
* **Design Intuition**: Verifies that the model has completely transitioned away from ROS 1 dependencies and established a proper ROS 2 node architecture.
* **Expected Outcome**: The source code must be devoid of any ROS 1 library references. It should successfully import the ROS 2 core client library and encapsulate the business logic within a class that inherits from the standard ROS 2 Node base class.

### Test 2: Atomic Polling Verification (Wait Logic)
* **Design Intuition**: Ensures the waiting logic is functional rather than an empty loop.
* **Expected Outcome**: The function must contain a loop that performs an active service call to the planning scene. Inside this loop, there must be an explicit comparison between the identifiers of objects existing in the world and the specific name of the target object.

### Test 3: Grasp Sequence Integrity
* **Design Intuition**: Enforces a strict operational order using sequence-based pattern matching. 
* **Expected Outcome**: The implementation must follow a specific chronological order: first, removing old objects to clear the scene; second, adding at least two distinct collision objects (the part and the table); and finally, calling the synchronization function before initiating the pickup action.

### Test 4: Async Fallback Mastery
* **Design Intuition**: This is the most difficult test, focusing on the model's ability to handle asynchronous branching.
* **Expected Outcome**: The code must demonstrate a complete asynchronous chain where the result of the first attempt is awaited. Upon detecting a failure through a conditional statement, it must initiate a second awaited call specifically targeting the "arm_torso" planning group, eventually returning a numerical status code.

### Test 5: Result Return Semantics
* **Design Intuition**: Ensures the final status of the operation is properly propagated.
* **Expected Outcome**: The function must explicitly extract the error code or status value from the action result object and return it to the caller, ensuring the action server can respond with the correct success or failure state.

---

## How to Run the Benchmark
Ensure `pytest` is installed in your ROS 2 environment, then execute:
```bash
python3 -m pytest src/task_006/test/test_oracle_ros2.py
```
