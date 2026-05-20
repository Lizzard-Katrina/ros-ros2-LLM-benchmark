# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):


# LLM Response

[FILENAME: husky_high_level_controller.py]
```python
#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class HuskyHighLevelController(Node):
    def __init__(self):
        super().__init__('husky_high_level_controller')

        # Parameters
        self.declare_parameter('distance_to_obstacle', 1.0)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.5)

        self.distance_to_obstacle = self.get_parameter('distance_to_obstacle').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # State variables
        self.current_pose = None
        self.laser_scan = None
        self.obstacle_detected = False

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            sensor_qos
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.distance_pub = self.create_publisher(
            Float64,
            '/distance_to_obstacle',
            10
        )

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Husky High Level Controller started')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        self.laser_scan = msg
        self.process_laser_scan()

    def process_laser_scan(self):
        if self.laser_scan is None:
            return

        # Find minimum distance in the front sector (within 30 degrees)
        min_distance = float('inf')
        angle_min = -math.pi / 6  # -30 degrees
        angle_max = math.pi / 6   # +30 degrees

        for i, distance in enumerate(self.laser_scan.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue

            angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
            if angle_min <= angle <= angle_max:
                if distance < min_distance:
                    min_distance = distance

        # Publish distance to obstacle
        distance_msg = Float64()
        distance_msg.data = min_distance if min_distance != float('inf') else self.laser_scan.range_max
        self.distance_pub.publish(distance_msg)

        # Check if obstacle is too close
        self.obstacle_detected = min_distance < self.distance_to_obstacle

    def control_loop(self):
        if self.current_pose is None:
            self.get_logger().warn('No odometry data received yet')
            return

        cmd_vel = Twist()

        if self.obstacle_detected:
            # Rotate in place to avoid obstacle
            cmd_vel.angular.z = self.angular_speed
            self.get_logger().info('Obstacle detected, rotating')
        else:
            # Move forward
            cmd_vel.linear.x = self.linear_speed
            self.get_logger().info('Moving forward')

        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = HuskyHighLevelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```