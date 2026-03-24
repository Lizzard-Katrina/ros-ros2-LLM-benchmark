#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from your_package.srv import AddThreeInts

class ClientNode(Node):
    def __init__(self):
        super().__init__("add_three_ints_client")
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')

    def send_request(self):
        # TODO: wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # END OF TODO
        req = AddThreeInts.Request()
        # populate request fields here if needed
        self.future = self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()
    client_node.send_request()
    while rclpy.ok():
        rclpy.spin_once(client_node)
        if client_node.future.done():
            try:
                response = client_node.future.result()
            except Exception as e:
                client_node.get_logger().info('Service call failed %r' % (e,))
            else:
                client_node.get_logger().info('Result of add_three_ints: for %d + %d + %d = %d' %
                                              (0, 0, 0, response.sum))
            break
    client_node.get_logger().info("Client executed.")
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()