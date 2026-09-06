#!/usr/bin/env python3
"""Simple node entry point for testing purposes."""
import rclpy
from rclpy.node import Node


class MoveBaseStateNode(Node):
    """Minimal node that demonstrates the MoveBaseState is available."""

    def __init__(self):
        super().__init__('move_base_state_node')
        self.get_logger().info('MoveBaseState node initialized (Nav2 NavigateToPose)')
        # Declare a parameter to confirm the node is alive
        self.declare_parameter('status', 'ready')


def main(args=None):
    rclpy.init(args=args)
    node = MoveBaseStateNode()
    try:
        rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()