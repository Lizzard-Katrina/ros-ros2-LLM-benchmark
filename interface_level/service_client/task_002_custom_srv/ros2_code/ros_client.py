#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from your_package.srv import AddThreeInts

def client_node():
    rclpy.init()
    node = rclpy.create_node("add_three_ints_client")
    
    client = node.create_client(AddThreeInts, 'add_three_ints')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
        
    req = AddThreeInts.Request()
    req.a = 1
    req.b = 2
    req.c = 3
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Result: %d' % future.result().sum)
    else:
        node.get_logger().error('Exception while calling service')

    node.get_logger().info("Client executed.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    client_node()