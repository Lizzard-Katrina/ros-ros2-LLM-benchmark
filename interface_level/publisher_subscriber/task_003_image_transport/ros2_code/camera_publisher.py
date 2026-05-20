#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# mock Image class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_publisher_node')
    
    pub = node.create_publisher(Image, 'camera/image', 10)
    
    rate = node.create_rate(10)
    while rclpy.ok():
        msg = Image()
        pub.publish(msg)
        rclpy.spin_once(node)
        rate.sleep()
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()