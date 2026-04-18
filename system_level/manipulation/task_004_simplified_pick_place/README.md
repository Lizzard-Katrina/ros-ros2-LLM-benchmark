# Task: EZ Pick-and-Place ROS 1 to ROS 2 Migration Benchmark

## 1. Brief Description
This task focuses on the migration of the **EZ Pick-and-Place** system from ROS 1 to ROS 2 (Foxy/Humble). The system is a hybrid manipulation pipeline that integrates **GraspIt!** (for grasp planning and contact physics) and **MoveIt 2** (for motion planning and collision checking). 

The goal is to ensure that the logic originally written in `rospy` and `tf` is correctly translated into `rclpy` and `tf2_ros`, maintaining strict synchronization between the two simulation engines.

---

## 2. Design Thinking for "Fill-in-the-Blanks" (Holes)

### A. Coordinate Scaling & Unit Conversion
* **Context:** MoveIt 2 (and ROS 2 in general) operates in **meters**, while the GraspIt! simulator backend is hardcoded to work in **millimeters**.
* **Design Logic:** A `pose_factor` (typically `1000`) must be applied to all translation values (x, y, z) before sending object/robot poses to GraspIt.
* **Critical implementation:** `p.position.x = trans.x * self.pose_factor`.

### B. Asynchronous Execution & Deadlock Prevention
* **Context:** In ROS 1, service calls were simple blocking proxies. In ROS 2, calling a service from a callback using a blocking `call()` will deadlock the SingleThreadedExecutor.
* **Design Logic:** The migration must use `call_async()` and `rclpy.spin_until_future_complete()`. This allows the node to continue processing executor events while waiting for the IK solver or database response.

### C. Kinematic Scene Consistency
* **Context:** When the robot successfully grasps an object, the object must be "attached" to the robot's end-effector in the MoveIt Planning Scene.
* **Design Logic:** The logic must call `self.moveit_scene.attach_object()` after a successful pick and `detach_object()` after a successful place. Failing to do so causes the motion planner to trigger a collision between the "held" object and the environment.

---

## 3. Oracle Test Design & Expected Code

The Oracle tests are designed to verify that the core ROS 2 architectural requirements and physical consistency are met.

### I. Infrastructure Migration (`test_migration_node_initialization`)
* **Goal:** Verify that `rospy` is completely purged and `rclpy` nodes/clients are correctly initialized.
* **Expected Code Snippet:**
    ```python
    self.node = rclpy.create_node('ez_pnp')
    self.add_model_srv = self.node.create_client(AddToDatabase, 'add_to_database')
    ```

### II. TF2 Buffer & Lookup Implementation (`test_migration_tf2_logic`)
* **Goal:** Ensure the transition from the legacy `tf` listener to the `tf2_ros` Buffer/Listener pattern.
* **Expected Code Snippet:**
    ```python
    self.tf2_buffer = tf2_ros.Buffer()
    self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self)
    # ... later in lookup ...
    now = rclpy.time.Time()
    trans = self.tf2_buffer.lookup_transform(target_frame, source_frame, now)
    ```

### III. Scaling Consistency (`test_migration_scaling_logic`)
* **Goal:** Verify that the 1000x scaling factor is applied during the transition to GraspIt's coordinate system.
* **Expected Code Snippet:**
    ```python
    loadm.model_pose.position.x = gripper_trans.transform.translation.x * self.pose_factor
    ```

### IV. Service Call Integrity (`test_migration_async_handling`)
* **Goal:** Check for the use of asynchronous patterns to prevent service deadlocks.
* **Expected Code Snippet:**
    ```python
    future = self.compute_ik_srv.call_async(req)
    rclpy.spin_until_future_complete(self.node, future)
    result = future.result()
    ```

### V. System-Level Integration (`test_integration_logic_params`)
* **Goal:** Ensure the test script (`test2_ez_pnp2.py`) targets the correct standardized object ("Z") and uses the correct move groups ("arm").
* **Expected Code Snippet:**
    ```python
    plan_req.graspit_target_object = "Z"
    plan_req.arm_move_group = "arm"
    ```
