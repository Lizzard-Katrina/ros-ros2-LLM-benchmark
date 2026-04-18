#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String


class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0


node = None


def callback(msg):
    # Print received data
    node.get_logger().info(f"Received: {msg}")


def main(args=None):
    global node
    rclpy.init(args=args)
    node = rclpy.create_node('person_subscriber')

    # Create a subscriber listening to /person_info
    subscription = node.create_subscription(
        String,
        '/person_info',
        callback,
        10
    )
    node.subscription = subscription

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()