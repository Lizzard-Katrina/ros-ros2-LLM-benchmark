#!/usr/bin/env python3
"""
ROS2 Camera Publisher – migrated from ROS1 image_transport publisher.

In ROS1 the image_transport library was used to advertise on /camera/image_raw.
In ROS2 Python there is no separate image_transport Python API; the idiomatic
equivalent is to publish sensor_msgs.msg.Image directly.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    """Publishes synthetic Image messages on /camera/image_raw."""

    def __init__(self):
        super().__init__('camera_publisher_node')

        # image_transport advertise equivalent – create a publisher for Image
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        # Timer replaces the ROS1 rate/sleep loop
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('CameraPublisher node started, publishing on /camera/image_raw')

    def timer_callback(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.width = 640
        msg.height = 480
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = 640 * 3
        msg.data = bytes(640 * 480 * 3)  # blank image data
        self.publisher_.publish(msg)
        self.get_logger().debug('Published an image')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()