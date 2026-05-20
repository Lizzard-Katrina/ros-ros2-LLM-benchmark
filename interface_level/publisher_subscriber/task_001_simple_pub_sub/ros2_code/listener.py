#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

def callback(data):
    # Keep logging logic
    rclpy.logging.get_logger('listener').info("I heard %s" % data.data)

def listener():
    rclpy.init()
    node = rclpy.create_node('listener')
    node.create_subscription(String, 'chatter', callback, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    listener()