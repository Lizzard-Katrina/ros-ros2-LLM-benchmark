"""A simple node that declares parameters, used for testing ParamClient."""
import rclpy
from rclpy.node import Node


class ParamServerNode(Node):
    def __init__(self):
        super().__init__('param_server_node')
        self.declare_parameter('test_param_str', 'hello')
        self.declare_parameter('test_param_int', 42)
        self.declare_parameter('test_param_float', 3.14)
        self.get_logger().info('ParamServerNode is spinning with declared parameters.')


def main(args=None):
    rclpy.init(args=args)
    node = ParamServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()