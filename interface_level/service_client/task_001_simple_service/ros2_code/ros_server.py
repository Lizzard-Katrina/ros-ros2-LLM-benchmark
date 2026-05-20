#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from beginner_tutorials.srv import AddTwoInts

def handle_add_two_ints(request, response):
    logger = rclpy.logging.get_logger('add_two_ints_server')
    logger.info("Server received request: %s + %s" % (request.a, request.b))
    response.sum = request.a + request.b
    return response

def server_node():
    rclpy.init()
    node = rclpy.create_node('add_two_ints_server')
    
    srv = node.create_service(AddTwoInts, 'add_two_ints', handle_add_two_ints)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    server_node()