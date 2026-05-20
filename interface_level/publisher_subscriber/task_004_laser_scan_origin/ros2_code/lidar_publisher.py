#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("lidar_publisher")

    publisher = node.create_publisher(LaserScan, '/scan', 10)
    rate = node.create_rate(10)

    while rclpy.ok():
        scan = LaserScan()

        scan.header.stamp = node.get_clock().now().to_msg()
        scan.header.frame_id = "laser_frame"
        
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        scan.ranges = [5.0] * 360
        scan.intensities = [100.0] * 360

        publisher.publish(scan)
        
        rclpy.spin_once(node, timeout_sec=0)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()