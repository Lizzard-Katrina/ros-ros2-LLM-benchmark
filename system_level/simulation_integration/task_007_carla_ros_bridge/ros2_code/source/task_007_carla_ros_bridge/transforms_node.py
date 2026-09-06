#!/usr/bin/env python3
"""
A minimal ROS2 node that exposes the transforms functionality
via a service for testing purposes.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray


class TransformsNode(Node):
    """Node that publishes twist conversions for testing."""

    def __init__(self):
        super().__init__('transforms_node')
        self.publisher_ = self.create_publisher(Twist, 'carla_twist', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('TransformsNode started')

    def timer_callback(self):
        """Publish a sample twist demonstrating the coordinate conversion."""
        twist = Twist()
        # Example: linear velocity (already in ROS frame)
        twist.linear.x = 1.0
        twist.linear.y = 2.0
        twist.linear.z = 3.0
        # Angular velocity converted from degrees to radians with handedness
        twist.angular.x = math.radians(10.0)
        twist.angular.y = -math.radians(20.0)
        twist.angular.z = -math.radians(30.0)
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TransformsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()