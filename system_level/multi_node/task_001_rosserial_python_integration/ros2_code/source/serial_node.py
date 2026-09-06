__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rclpy
from rclpy.node import Node
from SerialClient import SerialClient
from time import sleep
import sys


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('tcp_portnum', 11411)
        self.declare_parameter('fork_server', False)
        self.declare_parameter('fix_pyserial_for_test', False)

        # Retrieve parameters
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        fix_pyserial_for_test = self.get_parameter('fix_pyserial_for_test').value

        self.get_logger().info("Connecting to %s at %d baud" % (port, baud))

        # Instantiate SerialClient with dependency injection (pass self as node)
        self.client = SerialClient(self, port=port, baud=baud, fix_pyserial_for_test=fix_pyserial_for_test)


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()