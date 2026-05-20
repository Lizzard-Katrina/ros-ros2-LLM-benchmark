#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def callback(msg):
    if msg.ranges:
        closest = min(msg.ranges)
        print(f"Closest range: {closest}")
    else:
        print("No range data available")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("lidar_subscriber")

    subscriber = node.create_subscription(LaserScan, '/scan', callback, 10)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()