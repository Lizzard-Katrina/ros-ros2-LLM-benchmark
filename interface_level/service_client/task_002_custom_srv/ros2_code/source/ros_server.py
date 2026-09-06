#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from task_002_custom_srv.srv import AddThreeInts


def handle_add_three_ints(req, response):
    response.sum = req.a + req.b + req.c
    return response


class AddThreeIntsServer(Node):
    def __init__(self):
        super().__init__('add_three_ints_server')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.handle_add_three_ints)
        self.get_logger().info('Custom service server started.')

    def handle_add_three_ints(self, req, response):
        self.get_logger().info(f'Received request: {req.a} {req.b} {req.c}')
        response.sum = req.a + req.b + req.c
        return response


def server_node():
    rclpy.init()
    node = AddThreeIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    server_node()