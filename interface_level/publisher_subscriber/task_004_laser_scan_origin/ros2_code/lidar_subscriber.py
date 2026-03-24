#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanMock(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        # TODO: Process the incoming LaserScan message
        # Example: print the closest range value
        self.get_logger().info('Received scan with closest range value: %f' % min(msg.ranges))

def main(args=None):
    rclpy.init(args=args)
    laser_scan_mock = LaserScanMock()
    try:
        rclpy.spin(laser_scan_mock)
    except KeyboardInterrupt:
        laser_scan_mock.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        laser_scan_mock.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()