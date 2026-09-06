#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def callback(msg):
    if len(msg.ranges) > 0:
        closest = min(msg.ranges)
        print(f'Closest range: {closest:.4f} m')
    else:
        print('No range data received')


class LidarSubscriberNode(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', callback, 10)
        self.subscription  # prevent unused variable warning


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()