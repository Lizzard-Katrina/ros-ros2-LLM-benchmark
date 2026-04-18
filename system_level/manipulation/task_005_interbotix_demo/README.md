# Task 005: ROS 2 System Level Migration Benchmark (Python)

## 1. Brief Description
This task evaluates a Large Language Model's (LLM) ability to maintain **System-Level Architectural Consistency** during the migration of a robotic "Orchestration Script". 

The target file, `pick_place.py`, serves as the functional brain of the Interbotix XSArm perception system. It integrates three distinct functional modules:
* **Actuation:** `InterbotixManipulatorXS` (Arm control)
* **Perception:** `InterbotixPointCloudInterface` (Object detection)
* **Localization:** `InterbotixArmTagInterface` (Camera-to-Arm calibration)

In ROS 2, these modules can no longer rely on a global hidden state (like ROS 1's `rospy`). They must be explicitly linked through a shared **Node Instance** to ensure synchronized TF (Transform) buffers, consistent logging, and shared parameter access.

---
source code file:
```https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_perception/scripts/pick_place.py```


## 2. Hollowing Strategy (Logic for "Holes")
To test "System Level" competency rather than just syntax replacement, we use a **Deep Integration Hollowing** approach:

### Hole A: The Dependency Injection Bridge
* **Location:** The initialization block within the `main()` function.
* **Removed Code:** The instantiation of `bot`, `pcl`, and `armtag` objects along with the ROS node setup.
* **Migration Challenge:** The LLM must realize that creating three separate nodes (the "naive" approach) will lead to resource conflicts. It must implement **Dependency Injection** by creating one `rclpy.Node` and passing it to all three interfaces.

### Hole B: The Spatial Coordinate Contract
* **Location:** The perception-to-motion loop (coordinate lookup).
* **Removed Code:** The reference frame string (e.g., `wx200/base_link`) and the data destructuring logic.
* **Migration Challenge:** The LLM must adhere to the **ROS 2 TF Convention** (no leading slashes in frame names) and correctly map the data structure returned by the ROS 2 version of the perception API.

---

## 3. Test Design & Expected Outcomes (Oracle Strategy)

The Oracle suite uses pattern matching (RegEx) to validate that the migrated code satisfies the "System Contract."

### Test 1: Shared Node Instance (Architectural Consistency)
* **Design:** Captures the variable name of the created Node and verifies it is passed to every class constructor.
* **Expected Outcome:** `bot`, `pcl`, and `armtag` must all share the same node variable (e.g., `node=node_var`).

### Test 2: TF Naming Convention (Static Consistency)
* **Design:** Scans for coordinate frame strings like `ref_frame`.
* **Expected Outcome:** Strings must **not** start with a forward slash `/`. Failing this indicates a violation of ROS 2 static naming rules.

### Test 3: Constructor Parameter Mapping (API Correctness)
* **Design:** Validates the hardware identifier mapping in the new API.
* **Expected Outcome:** The `InterbotixManipulatorXS` constructor must correctly identify the hardware as `"wx200"`.

### Test 4: Anti-Leakage (Clean Migration)
* **Design:** Searches for legacy ROS 1 keywords.
* **Expected Outcome:** 0% presence of `rospy`, `rospy.init_node`, or `roslaunch`.

### Test 5: Lifecycle & Execution (API Correctness)
* **Design:** Checks for ROS 2 execution boilerplate.
* **Expected Outcome:** Presence of `rclpy.init()`, `rclpy.shutdown()`, and a spinning mechanism (e.g., `spin_once` or `spin`).

### Test 6: Semantic Data Extraction (Logic Consistency)
* **Design:** Matches how the script handles the output from the perception module.
* **Expected Outcome:** Correct indexing or destructuring (e.g., `centroid[0]` or `cluster['position']`) to prove the LLM understands the updated API return types.

---

## 4. Evaluation Criteria
A "Pass" is granted only if the LLM demonstrates **Global Context Awareness**:
1.  **Shared Resources:** One Node to rule them all.
2.  **Naming Protocol:** Strict adherence to ROS 2 string standards.
3.  **API Alignment:** Correct mapping of legacy parameters to the new ROS 2 Python SDK signatures.
