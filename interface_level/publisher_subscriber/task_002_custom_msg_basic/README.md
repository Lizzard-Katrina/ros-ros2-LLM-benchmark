## Oracle Test: Task 002 – ROS1 → ROS2 Custom Message Pub/Sub

### Test Case

This task evaluates whether a ROS1 publisher–subscriber example using a
custom message type is correctly translated into ROS2 Python code.

Two translated modules are tested:

- `publisher_node.py`
- `subscriber_node.py`

The custom message type is `task_002_custom_msg_basic/msg/Person`.

---

### Test Conditions

**Publisher (`publisher_node.py`)**

The test passes if all of the following conditions are met:

1. The module is importable as a Python module.
2. The module defines at least one class that subclasses `rclpy.node.Node`.
3. The node can be instantiated without runtime errors.
4. The node name is `"person_publisher"`.
5. The node creates at least one publisher with:
   - Topic name: `"person_info"`
   - Message type: `task_002_custom_msg_basic/msg/Person`

---

**Subscriber (`subscriber_node.py`)**

The test passes if all of the following conditions are met:

1. The module is importable as a Python module.
2. The module defines at least one class that subclasses `rclpy.node.Node`.
3. The node can be instantiated without runtime errors.
4. The node name is `"person_subscriber"`.
5. The node creates at least one subscription with:
   - Topic name: `"person_info"`
   - Message type: `task_002_custom_msg_basic/msg/Person`

---

### Expected Outcome

- **Pass**:  
  Both publisher and subscriber correctly use ROS2 node abstractions and
  establish communication using the custom `Person` message on topic
  `/person_info`.

- **Fail**:  
  The test fails if any required node abstraction, node name, publisher,
  subscription, or custom message type is missing or incorrect.
### How to run the test
- build the docker:
```docker build -t ros2-test .```
- run the docker:
```docker run -it --rm ros2-test```
- run testcase:
```python3 -m pytest src/task_002_custom_msg_basic/test/test_oracle_custom_msg.py```
