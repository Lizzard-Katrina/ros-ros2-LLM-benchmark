#!/usr/bin/env python3
"""Entry point for the talker node."""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        self.declare_parameter('topic_name', 'chatter')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f'Talker node started, publishing on "{topic_name}"')

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        time_float = current_time.sec + current_time.nanosec * 1e-9
        msg = String()
        msg.data = f'hello world {time_float}'
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()