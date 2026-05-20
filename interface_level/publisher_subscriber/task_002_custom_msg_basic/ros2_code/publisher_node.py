#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Mock of ROS1 custom message Person
class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('person_publisher')

    # ======= STUDENT TODO ========
    # Create a publisher named /person_info
    # publishing the custom Person message.
    # Fill the message fields and publish at 1 Hz.
    publisher = node.create_publisher(Person, '/person_info', 10)
    rate = node.create_rate(1.0)

    while rclpy.ok():
        msg = Person()

        # Fill the message fields: name, age, height
        # =============================
        msg.name = "John Doe"
        msg.age = 30
        msg.height = 180
        
        publisher.publish(msg)
        node.get_logger().info(f"Published: {msg.name}, {msg.age}, {msg.height}")
        
        rclpy.spin_once(node)
        rate.sleep()
        # END OF TODO

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()