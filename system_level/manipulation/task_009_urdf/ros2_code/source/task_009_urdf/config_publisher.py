"""
Simple node that publishes the robot_description parameter
from the URDF file, similar to robot_state_publisher usage.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path


class ConfigPublisher(Node):
    def __init__(self):
        super().__init__('config_publisher')

        # Locate config files relative to this package's share directory
        pkg_share = Path(__file__).resolve().parents[1]
        urdf_path = pkg_share / 'arm_urdf.urdf'
        srdf_path = pkg_share / 'manipulator.srdf'
        limits_path = pkg_share / 'joint_limits.yaml'

        # Try share directory if local doesn't exist
        if not urdf_path.exists():
            from ament_index_python.packages import get_package_share_directory
            pkg_share = Path(get_package_share_directory('task_009_urdf')) / 'config'
            urdf_path = pkg_share / 'arm_urdf.urdf'
            srdf_path = pkg_share / 'manipulator.srdf'
            limits_path = pkg_share / 'joint_limits.yaml'

        self.urdf_content = urdf_path.read_text() if urdf_path.exists() else ''
        self.srdf_content = srdf_path.read_text() if srdf_path.exists() else ''
        self.limits_content = limits_path.read_text() if limits_path.exists() else ''

        self.declare_parameter('robot_description', self.urdf_content)
        self.declare_parameter('robot_description_semantic', self.srdf_content)

        self.pub_urdf = self.create_publisher(String, 'robot_description', 10)
        self.pub_srdf = self.create_publisher(String, 'robot_description_semantic', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('ConfigPublisher started.')

    def timer_callback(self):
        msg = String()
        msg.data = self.urdf_content
        self.pub_urdf.publish(msg)

        msg2 = String()
        msg2.data = self.srdf_content
        self.pub_srdf.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = ConfigPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()