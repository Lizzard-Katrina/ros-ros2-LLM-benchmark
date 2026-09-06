#!/usr/bin/env python3
"""
A minimal ROS 2 node that uses the interface_finder module to demonstrate
the graph API usage. Used for runtime testing.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InterfaceFinderNode(Node):
    def __init__(self):
        super().__init__('interface_finder_node')
        self.publisher_ = self.create_publisher(String, '~/found_topics', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('InterfaceFinderNode started')

    def timer_callback(self):
        from task_005_KIE_multiagent_model.interface_finder import _get_topic_from_node
        result = _get_topic_from_node(self, 'String', wait=False, check_host=False)
        msg = String()
        msg.data = ','.join(result) if result else ''
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceFinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()