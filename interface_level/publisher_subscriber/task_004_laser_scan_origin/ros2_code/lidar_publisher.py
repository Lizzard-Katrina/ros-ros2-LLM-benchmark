#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node("lidar_publisher")

    # TODO: Create a publisher that publishes to /scan using LaserScan
    # Fill in all LaserScan fields in the while loop
    # publish message afterwards
    publisher = node.create_publisher(LaserScan, "/scan", 10)

    rate = node.create_rate(10)

    try:
        while rclpy.ok():
            scan = LaserScan()

            scan.header.stamp = node.get_clock().now().to_msg()
            scan.header.frame_id = "laser_frame"

            # Angular limits
            scan.angle_min = -1.57
            scan.angle_max = 1.57
            scan.angle_increment = 0.01

            # Timing
            scan.time_increment = 0.0
            scan.scan_time = 0.1

            # Range limits
            scan.range_min = 0.12
            scan.range_max = 10.0

            # Data
            num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
            scan.ranges = [1.0] * num_readings
            scan.intensities = [100.0] * num_readings

            publisher.publish(scan)

            rate.sleep()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # END OF TODO


if __name__ == "__main__":
    main()
