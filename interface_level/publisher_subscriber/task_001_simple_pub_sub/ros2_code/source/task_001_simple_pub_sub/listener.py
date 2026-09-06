#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def callback(msg):
    print('I heard: [%s]' % msg.data)


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'chatter', callback, 10)


def listener():
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    listener()


if __name__ == '__main__':
    main()