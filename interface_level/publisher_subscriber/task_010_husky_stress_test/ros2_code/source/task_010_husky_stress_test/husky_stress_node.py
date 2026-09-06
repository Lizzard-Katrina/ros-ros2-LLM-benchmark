#!/usr/bin/env python3
"""
ROS2 node that simulates the high-frequency sensor publishers
defined in the Husky Gazebo URDF/Xacro:
  - IMU data at 100 Hz on /imu/data
  - GPS/NavSat fix at 10 Hz on /navsat/fix
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Header
import math
import time


class HuskyStressNode(Node):
    def __init__(self):
        super().__init__('husky_stress_node')

        # IMU publisher at 100 Hz
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.imu_timer = self.create_timer(1.0 / 100.0, self.publish_imu)

        # GPS / NavSat publisher at 10 Hz
        self.gps_pub = self.create_publisher(NavSatFix, 'navsat/fix', 10)
        self.gps_timer = self.create_timer(1.0 / 10.0, self.publish_gps)

        self.imu_seq = 0
        self.gps_seq = 0

        self.get_logger().info('HuskyStressNode started: IMU@100Hz, GPS@10Hz')

    def publish_imu(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulated IMU data: gravity on z-axis, no rotation
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        self.imu_pub.publish(msg)
        self.imu_seq += 1

    def publish_gps(self):
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'navsat_link'

        # Simulated GPS fix: Clearpath HQ coordinates
        msg.latitude = 43.4723
        msg.longitude = -80.5449
        msg.altitude = 334.0

        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS

        self.gps_pub.publish(msg)
        self.gps_seq += 1


def main(args=None):
    rclpy.init(args=args)
    node = HuskyStressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()