#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        self.get_logger().info('Client node running')

    def call_service(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def client_node():
    rclpy.init()
    node = AddTwoIntsClient()
    try:
        result = node.call_service(2, 3)
        node.get_logger().info('Result: %d' % result.sum)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    client_node()


if __name__ == '__main__':
    main()