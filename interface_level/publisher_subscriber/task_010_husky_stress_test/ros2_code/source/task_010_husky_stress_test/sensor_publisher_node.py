"""
ROS2 node that simulates the Husky sensor publishers defined in husky.gazebo.xacro.

This node publishes:
  - IMU data on /imu/data at 50 Hz (sensor_msgs/Imu)
  - GPS fix on /navsat/fix at 10 Hz (sensor_msgs/NavSatFix)
  - GPS velocity on /navsat/vel at 10 Hz (geometry_msgs/TwistStamped)

These correspond to the Gazebo plugin publishers defined in the URDF/Xacro.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
import math
import time


class HuskySensorPublisher(Node):
    """Simulates Husky sensor publishers (IMU + GPS) as defined in husky.gazebo.xacro."""

    def __init__(self):
        super().__init__('husky_sensor_publisher')

        # IMU publisher at 50 Hz (matching updateRate in xacro)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.imu_timer = self.create_timer(1.0 / 50.0, self.publish_imu)

        # GPS fix publisher at 10 Hz (matching updateRate in xacro)
        self.gps_pub = self.create_publisher(NavSatFix, 'navsat/fix', 10)
        self.gps_timer = self.create_timer(1.0 / 10.0, self.publish_gps)

        # GPS velocity publisher at 10 Hz
        self.gps_vel_pub = self.create_publisher(TwistStamped, 'navsat/vel', 10)
        self.gps_vel_timer = self.create_timer(1.0 / 10.0, self.publish_gps_vel)

        self.imu_seq = 0
        self.gps_seq = 0

        self.get_logger().info('Husky sensor publisher started (IMU@50Hz, GPS@10Hz)')

    def publish_imu(self):
        """Publish IMU data matching the imu_controller plugin output."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulated orientation (identity quaternion)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        # Simulated angular velocity (near zero with noise)
        msg.angular_velocity.x = 0.001 * math.sin(self.imu_seq * 0.01)
        msg.angular_velocity.y = 0.001 * math.cos(self.imu_seq * 0.01)
        msg.angular_velocity.z = 0.0

        # Simulated linear acceleration (gravity on z)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        self.imu_pub.publish(msg)
        self.imu_seq += 1

    def publish_gps(self):
        """Publish GPS NavSatFix matching the gps_controller plugin output."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'navsat_link'

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Reference coordinates from xacro
        msg.latitude = 49.9
        msg.longitude = 8.9
        msg.altitude = 0.0

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance[0] = 1.0
        msg.position_covariance[4] = 1.0
        msg.position_covariance[8] = 1.0

        self.gps_pub.publish(msg)
        self.gps_seq += 1

    def publish_gps_vel(self):
        """Publish GPS velocity matching the gps_controller plugin output."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'navsat_link'

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        self.gps_vel_pub.publish(msg)


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