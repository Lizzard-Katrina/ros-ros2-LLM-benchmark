## Task 004: LaserScan Origin

This task evaluates the semantic correctness of translating ROS1
LaserScan publisher and subscriber code into ROS2 Python code.

The oracle tests validate structure and intent, not runtime execution.

### Reference source:

an ROS2 package github file : ```https://github.com/ros-drivers/urg_node```

## Oracle Test: Lidar Publisher

### File Under Test
- `lidar_publisher.py`

### Checks
1. The module can be imported.
2. At least one class subclasses `rclpy.node.Node`.
3. The node class is instantiable.(if rclpy.init()` has been called.)
4. A `LaserScan` message can be constructed.
5. The `LaserScan` message contains key fields:
   - `ranges`
   - `angle_min`
   - `angle_max`

### Not Required
- Real ROS2 message type support
- Actual publishing
- Time stamping
- `rclpy.spin()`

### Expected Outcome
- The translated code constructs and populates a LaserScan message
  in a ROS2-style node.

## Oracle Test: Lidar Subscriber

### File Under Test
- `lidar_subscriber.py`

### Checks
1. The module can be imported.
2. A class subclasses `rclpy.node.Node`.
3. A callback function named `callback(msg)` exists.

### Not Required
- Real message reception
- Range computation correctness
- Runtime spinning

### Expected Outcome
- The translated code subscribes to LaserScan messages
  and provides a valid processing callback.
## Mocking Policy

- `LaserScan` is mocked when ROS2 message support is unavailable.
- The mock mirrors the ROS message field structure.
- This ensures oracle tests validate translation semantics,
  not ROS2 build configuration.
## Mocking Policy

- `LaserScan` is mocked when ROS2 message support is unavailable.
- The mock mirrors the ROS message field structure.
- This ensures oracle tests validate translation semantics,
  not ROS2 build configuration.
