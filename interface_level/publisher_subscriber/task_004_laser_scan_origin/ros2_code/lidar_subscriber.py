#!/usr/bin/env python3
import math
import rclpy
from rclpy.logging import get_logger
from sensor_msgs.msg import LaserScan as LaserScanMsg

# test/mocks/laser_scan_mock.py


class Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class LaserScan:
    def __init__(self):
        self.header = Header()

        # Angular limits
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

        # Timing
        self.time_increment = 0.0
        self.scan_time = 0.0

        # Range limits
        self.range_min = 0.0
        self.range_max = 0.0

        # Data
        self.ranges = []
        self.intensities = []


def callback(msg):
    # TODO: Process the incoming LaserScan message
    # Example: print the closest range value
    valid_ranges = [r for r in msg.ranges if math.isfinite(r) and r >= msg.range_min and r <= msg.range_max]
    if valid_ranges:
        closest = min(valid_ranges)
        get_logger("lidar_subscriber").info(f"Closest range: {closest:.3f} m")
    else:
        get_logger("lidar_subscriber").info("No valid range readings in scan.")


def main():
    rclpy.init()
    node = rclpy.create_node("lidar_subscriber")

    # Create a subscriber for /scan topic using LaserScan
    node.create_subscription(LaserScanMsg, "/scan", callback, 10)

    rclpy.spin(node)
    # END OF TODO
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()