#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import numpy as np

class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # TODO: use image_transport to construct publisher
        # and insert information of Image
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Image, 'camera/image', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        msg = Image()
        msg.width = 640
        msg.height = 480
        msg.encoding = "rgb8"
        msg.data = np.zeros((msg.height * msg.width * 3), dtype=np.uint8).tobytes()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()