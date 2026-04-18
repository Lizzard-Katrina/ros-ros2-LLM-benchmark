#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

node = None


def callback(data):
    # Keep logging logic
    node.get_logger().info("I heard %s" % data.data)


def listener():
    global node
    # TODO: initialize node 'listener'
    # and subscribe to topic 'chatter'
    # and keep spin
    # END OF TODO
    rclpy.init()
    node = Node('listener')
    node.create_subscription(String, 'chatter', callback, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    listener()