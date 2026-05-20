#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from your_package.srv import AddThreeInts

def handle_add_three_ints(request, response):
    logger = rclpy.logging.get_logger("add_three_ints_server")
    logger.info("Received request: %s %s %s" % (request.a, request.b, request.c))
    response.sum = request.a + request.b + request.c
    return response

def server_node():
    rclpy.init()
    node = rclpy.create_node("add_three_ints_server")
    
    srv = node.create_service(AddThreeInts, 'add_three_ints', handle_add_three_ints)
    node.get_logger().info("Custom service server started.")
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    server_node()