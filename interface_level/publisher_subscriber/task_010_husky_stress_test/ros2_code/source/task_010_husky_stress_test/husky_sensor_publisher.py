"""
ROS2 translation of Husky Gazebo sensor publishers.

This node replicates the high-frequency sensor publishing behavior originally
defined via Gazebo plugins in husky.gazebo.xacro:
  - IMU data on /imu/data at 50 Hz
  - GPS/NavSat fix on /navsat/fix at 10 Hz
  - GPS velocity on /navsat/vel at 10 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
import math
import time


class HuskySensorPublisher(Node):
    """
    Publishes simulated IMU and GPS sensor data at high frequency,
    mirroring the Gazebo plugin behavior from the original ROS1 Husky package.
    """

    def __init__(self):
        super().__init__('husky_sensor_publisher')

        # Declare parameters with defaults matching the original xacro
        self.declare_parameter('imu_update_rate', 50.0)
        self.declare_parameter('gps_update_rate', 10.0)

        imu_rate = self.get_parameter('imu_update_rate').get_parameter_value().double_value
        gps_rate = self.get_parameter('gps_update_rate').get_parameter_value().double_value

        # IMU publisher - high frequency (50 Hz default)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        imu_period = 1.0 / imu_rate
        self.imu_timer = self.create_timer(imu_period, self.imu_callback)

        # GPS/NavSat fix publisher (10 Hz default)
        self.navsat_fix_pub = self.create_publisher(NavSatFix, 'navsat/fix', 10)
        gps_period = 1.0 / gps_rate
        self.gps_fix_timer = self.create_timer(gps_period, self.gps_fix_callback)

        # GPS velocity publisher (10 Hz default)
        self.navsat_vel_pub = self.create_publisher(TwistStamped, 'navsat/vel', 10)
        self.gps_vel_timer = self.create_timer(gps_period, self.gps_vel_callback)

        self.get_logger().info(
            f'Husky sensor publisher started: IMU@{imu_rate}Hz, GPS@{gps_rate}Hz'
        )

    def imu_callback(self):
        """Publish simulated IMU data (mirrors imu_controller Gazebo plugin)."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Identity orientation (robot at rest)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        # Zero angular velocity
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # Gravity in z for linear acceleration (sensor at rest)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        self.imu_pub.publish(msg)

    def gps_fix_callback(self):
        """Publish simulated NavSat fix (mirrors gps_controller Gazebo plugin)."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'navsat_link'

        # Default coordinates (Clearpath HQ approximate)
        msg.latitude = 43.4723
        msg.longitude = -80.5449
        msg.altitude = 334.0

        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS

        self.navsat_fix_pub.publish(msg)

    def gps_vel_callback(self):
        """Publish simulated GPS velocity."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'navsat_link'

        # Zero velocity (robot at rest)
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        self.navsat_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HuskySensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()