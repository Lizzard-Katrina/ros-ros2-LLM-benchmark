#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


class LatchedPubSubNode(Node):
    def __init__(self):
        super().__init__('latched_pub_sub_node')

        # QoS profile with transient local durability (equivalent to ROS1 latch=True)
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Create a latched publisher on 'latched_topic'
        self.publisher = self.create_publisher(String, 'latched_topic', qos)

        # Create a subscriber on 'latched_topic' with matching QoS
        self.subscription = self.create_subscription(
            String, 'latched_topic', self.callback, qos
        )

        # Publish a single message after a short delay
        self.timer = self.create_timer(0.5, self.publish_once)
        self.published = False

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def publish_once(self):
        if not self.published:
            msg = String()
            msg.data = 'Hello, latched world!'
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            self.published = True
            # Cancel the timer after publishing once
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = LatchedPubSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()