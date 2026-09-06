#!/usr/bin/env python3
"""Minimal node entry point."""
import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('gazebo_interface_node')
    node.get_logger().info('gazebo_interface_node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()