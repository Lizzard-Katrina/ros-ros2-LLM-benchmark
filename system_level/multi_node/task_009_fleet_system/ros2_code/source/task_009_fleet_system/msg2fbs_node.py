"""
A minimal ROS 2 node that wraps the msg2fbs schema generation logic
and publishes the generated schema on a topic for verification.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import the generation utilities
from task_009_fleet_system.msg2fbs_lib import generate_schema_lines


class Msg2FbsNode(Node):
    def __init__(self):
        super().__init__('msg2fbs_node')
        self.publisher_ = self.create_publisher(String, 'fbs_schema', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('msg2fbs_node started')

    def timer_callback(self):
        lines = list(generate_schema_lines())
        msg = String()
        msg.data = '\n'.join(lines)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Msg2FbsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()