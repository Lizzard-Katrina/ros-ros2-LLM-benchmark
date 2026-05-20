__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rclpy
from rclpy.node import Node
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing

import sys

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 57600)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        self.get_logger().info(f"Connecting to {port} at {baud} baud")
        self.client = SerialClient(self, port=port, baud=baud)
        
    def run(self):
        self.client.run()

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    
    try:
        # Run the client in a separate thread or just spin if client handles its own threads
        import threading
        client_thread = threading.Thread(target=node.run)
        client_thread.start()
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.txStopRequest()
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()