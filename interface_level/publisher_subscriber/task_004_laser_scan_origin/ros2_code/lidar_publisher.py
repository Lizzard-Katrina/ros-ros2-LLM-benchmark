#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

class LaserScanMock(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        # QoS Profile
        qos_profile = QoSProfile(depth=10)

        # TODO: Create a publisher that publishes to /scan using LaserScan
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile)

        # Timer
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        scan = LaserScan()

        # Fill in all LaserScan fields
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -3.14159
        scan.angle_max = 3.14159
        scan.angle_increment = 0.01745
        scan.time_increment = 0.00001
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = [1.0] * 360
        scan.intensities = [1.0] * 360

        # Publish message
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    laser_scan_mock = LaserScanMock()

    try:
        rclpy.spin(laser_scan_mock)
    except KeyboardInterrupt:
        pass

    laser_scan_mock.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()