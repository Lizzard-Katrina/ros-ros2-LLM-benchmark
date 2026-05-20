#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('talker')
    publisher = node.create_publisher(String, 'chatter', 10)
    
    rate = node.create_rate(1)
    while rclpy.ok():
        msg = String()
        msg.data = "hello from ros2"
        publisher.publish(msg)
        rclpy.spin_once(node)
        rate.sleep()
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()