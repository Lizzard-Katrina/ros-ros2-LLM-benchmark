#!/usr/bin/env python3
"""
A mock camera driver node at /head_camera/driver that declares
auto_exposure and auto_white_balance parameters and serves
set_parameters requests.
"""
import rclpy
from rclpy.node import Node


class MockCameraDriverNode(Node):
    def __init__(self):
        super().__init__('driver', namespace='head_camera')
        # Declare the parameters with default values
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('auto_white_balance', True)
        self.get_logger().info('MockCameraDriverNode ready at /head_camera/driver')


def main():
    rclpy.init()
    node = MockCameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()