#!/usr/bin/env python
import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('sim_monitor_node')
    node.get_logger().info('Sim monitor node started (placeholder).')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()