## Task 003: Image Transport

This task evaluates the semantic correctness of translating ROS1 image
publisher/subscriber code into ROS2 Python code.

The oracle tests are static and do not require runtime execution,
ROS2 package metadata, or build configuration.

## Oracle Test: Camera Publisher

### File Under Test
- `camera_publisher.py`

### Checks
1. The module can be imported.
2. At least one class subclasses `rclpy.node.Node`.
3. The Node subclass is instantiable.
4. The node creates a publisher with message type:
   - `sensor_msgs.msg.Image`

### Not Required
- `image_transport` Python API
- Runtime publishing
- `rclpy.spin()`
- ROS2 package / `setup.py` / `CMakeLists.txt`

### Expected Outcome
- A ROS2 Node subclass exists and publishes `Image` messages.
## Oracle Test: Camera Subscriber

### File Under Test
- `camera_subscriber.py`

### Checks
1. The module can be imported.
2. At least one class subclasses `rclpy.node.Node`.
3. The node creates a subscription with message type:
   - `sensor_msgs.msg.Image`
4. A callback function is provided to the subscription.

### Not Required
- `image_transport` Python API
- Actual image reception
- Runtime spinning

### Expected Outcome
- A ROS2 Node subclass exists and subscribes to `Image` messages.
## Oracle Interpretation Rules

- Direct use of `sensor_msgs.msg.Image` is considered a valid
  image_transport translation.

- Importing or depending on a Python module named `image_transport`
  is considered an incorrect translation.

- Import errors caused by unavailable ROS2 Python APIs indicate
  translation errors, not oracle test design issues.
