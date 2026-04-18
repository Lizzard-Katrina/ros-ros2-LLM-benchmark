#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String as ROSString


# mock string class
class String:
    def __init__(self):
        self.data = ""


def main():
    rclpy.init()
    node = rclpy.create_node("chatter_publisher")
    publisher = node.create_publisher(ROSString, "/chatter", 10)

    rate = node.create_rate(1.0)
    while rclpy.ok():
        msg = ROSString()
        msg.data = "hello from ros1"
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()