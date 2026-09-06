#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from task_002_custom_srv.srv import AddThreeInts


class AddThreeIntsClient(Node):
    def __init__(self):
        super().__init__('add_three_ints_client')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b, c):
        request = AddThreeInts.Request()
        request.a = a
        request.b = b
        request.c = c
        future = self.cli.call_async(request)
        return future


def client_node():
    rclpy.init()
    node = AddThreeIntsClient()
    future = node.send_request(1, 2, 3)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is not None:
        node.get_logger().info(f'Result: {future.result().sum}')
    else:
        node.get_logger().error('Service call failed')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    client_node()