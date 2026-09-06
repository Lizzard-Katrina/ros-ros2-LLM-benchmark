"""
Minimal ROS 2 node that represents the exploration/mapping pipeline.
In a real system this would coordinate SLAM Toolbox and Nav2 components.
Here it serves as a placeholder entry point for the translated package.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_mapping_node')
        self.get_logger().info('Exploration mapping node started (ROS 2 Humble)')

        # Declare parameters that mirror the costmap config
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('update_frequency', 5.0)
        self.declare_parameter('publish_frequency', 2.0)
        self.declare_parameter('observation_sources', 'laser')

        # Publisher to signal status
        self.status_pub = self.create_publisher(String, 'exploration_status', 10)
        # Publish at 2 Hz
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'exploration_active'
        self.status_pub.publish(msg)
        self.get_logger().info('Published exploration_active')


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()