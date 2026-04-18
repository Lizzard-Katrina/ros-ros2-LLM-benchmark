#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Image

try:
    from image_transport import ImageTransport
except ImportError:
    from image_transport_py import ImageTransport

_node = None


def callback(msg):
    # image_transport-based callback
    _node.get_logger().info("Received an image")


def main():
    global _node
    rclpy.init()
    _node = rclpy.create_node('camera_subscriber_node')

    # use image_transport to construct subscriber
    image_transport = ImageTransport(_node)
    sub = image_transport.subscribe('camera/image', callback, 'raw')

    rclpy.spin(_node)

    _node.destroy_node()
    rclpy.shutdown()
    # END OF TODO


if __name__ == '__main__':
    main()
