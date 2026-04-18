#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

def talker():
    # TODO: create a ROS publisher for topic 'chatter'
    # initialize node 'talker'
    rclpy.init()
    node = Node('talker')
    publisher = node.create_publisher(String, 'chatter', 10)
    # END OF TODO
    rate = node.create_rate(1)  # Keep this line

    while rclpy.ok():
        msg = "Hello world %s" % (node.get_clock().now().nanoseconds / 1e9)   # Keep message logic
        node.get_logger().info(msg)

        # TODO: publish the message
        publisher.publish(String(data=msg))
        #END OF TODO
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        talker()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass