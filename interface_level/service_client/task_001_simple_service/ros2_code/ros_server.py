#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from beginner_tutorials.srv import AddTwoInts

_node = None


def handle_add_two_ints(req, response):
    # TODO: AI/user completes service logic
    _node.get_logger().info(f"Server received request: {req.a} + {req.b}")
    response.sum = req.a + req.b
    return response
    # END OF TODO


def server_node():
    rclpy.init()
    global _node
    _node = rclpy.create_node('add_two_ints_server')
    # TODO: advertise the service
    _node.create_service(AddTwoInts, 'add_two_ints', handle_add_two_ints)
    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()
    # END OF TODO


if __name__ == "__main__":
    server_node()