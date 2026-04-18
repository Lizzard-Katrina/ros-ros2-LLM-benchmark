#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Image as RosImage
from image_transport_py import ImageTransport


# mock Image class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''


def main():
    rclpy.init()
    node = rclpy.create_node('camera_publisher_node')

    # TODO: use image_transport to construct publisher
    # and insert information of Image
    image_transport = ImageTransport(node)
    pub = image_transport.advertise('camera/image', 10)

    rate = node.create_rate(10)
    while rclpy.ok():
        msg = RosImage()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.width = Image.width
        msg.height = Image.height
        msg.encoding = Image.encoding
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = Image.data if Image.data else bytes(msg.height * msg.step)

        pub.publish(msg)
        rate.sleep()
    # END OF TODO

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
