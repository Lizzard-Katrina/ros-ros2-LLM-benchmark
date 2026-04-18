#!/usr/bin/env python3
import rclpy

try:
    from person_interfaces.msg import Person
except ImportError:
    # Fallback mock if custom interface package is unavailable
    class Person:
        def __init__(self):
            self.name = ""
            self.age = 0
            self.height = 0


def main():
    rclpy.init()
    node = rclpy.create_node('person_publisher')

    publisher = node.create_publisher(Person, '/person_info', 10)
    rate = node.create_rate(1.0)

    while rclpy.ok():
        msg = Person()

        msg.name = "Tom"
        msg.age = 18
        msg.height = 175

        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
