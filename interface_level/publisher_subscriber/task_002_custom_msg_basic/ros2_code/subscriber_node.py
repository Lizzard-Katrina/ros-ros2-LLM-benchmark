#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0

node = None

def callback(msg):
    # TODO
    # Print received data
    global node
    if node is not None:
        node.get_logger().info(f"Received: {msg.name}, {msg.age}, {msg.height}")

def main(args=None):
    global node
    rclpy.init(args=args)
    node = rclpy.create_node('person_subscriber')

    # Create a subscriber listening to /person_info
    subscriber = node.create_subscription(Person, '/person_info', callback, 10)

    rclpy.spin(node)
    #END OF TODO
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()