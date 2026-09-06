#!/usr/bin/env python3
"""
ROS2 Camera Subscriber – migrated from ROS1 image_transport subscriber.

In ROS1 the image_transport library was used to subscribe to /camera/image_raw.
In ROS2 Python the idiomatic equivalent is to create_subscription for
sensor_msgs.msg.Image directly.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraSubscriber(Node):
    """Subscribes to Image messages on /camera/image_raw."""

    def __init__(self):
        super().__init__('camera_subscriber_node')

        # image_transport subscribe equivalent – create a subscription for Image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10,
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('CameraSubscriber node started, listening on /camera/image_raw')

    def callback(self, msg):
        """Callback invoked when an Image message is received."""
        self.get_logger().info(
            f'Received an image: {msg.width}x{msg.height}, encoding={msg.encoding}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()