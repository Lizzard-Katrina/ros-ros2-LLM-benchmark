#!/usr/bin/env python3
"""A minimal node that hosts a service and runs a ServiceState against it."""
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import threading
import time
import sys


def main():
    rclpy.init()
    node = Node('service_state_test_node')

    # Create a simple service
    def handle_set_bool(request, response):
        response.success = request.data
        response.message = 'ok' if request.data else 'not ok'
        return response

    srv = node.create_service(SetBool, '/test_set_bool', handle_set_bool)

    # Spin in a thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.get_logger().info('Service state test node is running.')

    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()