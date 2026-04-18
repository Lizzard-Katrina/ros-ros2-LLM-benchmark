__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.get_logger().info("SerialNode initialized")

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 57600)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        self.serial_client = SerialClient(self.port, self.baud, node=self)

        self.serial_server = RosSerialServer(11411)
        self.serial_server.listen()

        self.timer = self.create_timer(0.1, self.spin)

    def spin(self):
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)

    node = SerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    except ExternalShutdownException:
        node.get_logger().info("Received shutdown request")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()