# ROS2 equivalent scaffold
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        # TODO: topic remapping handled in launch
        self.pub = self.create_publisher(String, '_____', 10)

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        # TODO: topic remapping handled in launch
        self.sub = self.create_subscription(String, '_____', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
