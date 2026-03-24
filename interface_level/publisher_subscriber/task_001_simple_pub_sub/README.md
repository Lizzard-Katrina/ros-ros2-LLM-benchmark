# Task 001 — Simple Publisher & Subscriber (C++)

### **Objective**
This task evaluates whether a model can translate a basic ROS1 C++ talker/listener pair into an equivalent ROS2 Foxy implementation.

The ROS1 code is intentionally left empty, because this is a well-known canonical example.  
The expected ROS2 code is provided in `expected_ros2_code/`.

## Oracle Test: Task 001 – ROS1 → ROS2 Pub/Sub Translation

### Test Case

This oracle test checks whether the ROS1 publisher–subscriber example is
correctly translated into ROS2 Python code.

Two translated modules are tested independently:

- `listener.py` (subscriber)
- `talker.py` (publisher)

The test imports the translated modules and inspects their ROS2 runtime
objects using `rclpy`.

---

### Test Conditions

**Listener (`listener.py`)**

The test passes if all of the following conditions are met:

1. The module defines at least one class that subclasses `rclpy.node.Node`.
2. The node can be instantiated without runtime errors.
3. The node name is `"listener"`.
4. The node creates at least one subscription with:
   - Topic name: `"chatter"`
   - Message type: `std_msgs/msg/String`

---

**Talker (`talker.py`)**

The test passes if all of the following conditions are met:

1. The module is importable as a Python module.
2. The module defines at least one class that subclasses `rclpy.node.Node`.
3. The node can be instantiated without runtime errors.
4. The node name is `"talker"`.
5. The node creates at least one publisher with:
   - Topic name: `"chatter"`
   - Message type: `std_msgs/msg/String`

---

### Expected Outcome

- **Pass**:  
  All conditions listed above are satisfied for both listener and talker.

- **Fail**:  
  The test fails if any required node abstraction, node name, publisher,
  subscription, or module import condition is not satisfied.
