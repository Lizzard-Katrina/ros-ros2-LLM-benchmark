#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ParamProvider(Node):
    def __init__(self):
        super().__init__("param_provider")
        self.declare_parameter("robot.speed", 1.5)
        self.declare_parameter("robot.name", "initial_robot")
        self.declare_parameter("robot", "parent_initial")


def main():
    rclpy.init()
    node = ParamProvider()
    print("param_provider_ready", flush=True)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()