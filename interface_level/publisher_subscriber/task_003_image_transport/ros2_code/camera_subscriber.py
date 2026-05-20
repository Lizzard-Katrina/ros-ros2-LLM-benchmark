#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

#mock Image Class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

def callback(msg):
    rclpy.logging.get_logger('camera_subscriber_node').info("Received an image")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_subscriber_node')
    
    sub = node.create_subscription(Image, 'camera/image', callback, 10)

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()