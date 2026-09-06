#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.handle_add_two_ints)
        self.get_logger().info('Server node running')

    def handle_add_two_ints(self, request, response):
        self.get_logger().info('Server received request: %d + %d' % (request.a, request.b))
        response.sum = request.a + request.b
        return response


def server_node():
    rclpy.init()
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    server_node()


if __name__ == '__main__':
    main()