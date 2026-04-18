#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from your_package.srv import AddThreeInts


def client_node():
    rclpy.init()
    node = rclpy.create_node("add_three_ints_client")
    # TODO: wait for service
    client = node.create_client(AddThreeInts, "add_three_ints")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
    # call service
    request = AddThreeInts.Request()
    request.a = 1
    request.b = 2
    request.c = 3

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    # END OF TODO
    node.get_logger().info("Client executed.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    client_node()