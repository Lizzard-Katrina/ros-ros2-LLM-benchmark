#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarPublisherNode(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.12
        scan.range_max = 3.5

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [1.0] * num_readings
        scan.intensities = [100.0] * num_readings

        self.publisher_.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()