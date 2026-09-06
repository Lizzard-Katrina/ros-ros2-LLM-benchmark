"""Simple entry point node for testing."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class MasterMonitorNode(Node):
    def __init__(self):
        super().__init__('master_monitor_node')
        self.get_logger().info('MasterMonitorNode started')
        # Publish a heartbeat on a topic for testing
        self.pub = self.create_publisher(String, 'master_monitor/heartbeat', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'alive_%f' % time.time()
        self.pub.publish(msg)
        self.get_logger().debug('Published heartbeat: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MasterMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()