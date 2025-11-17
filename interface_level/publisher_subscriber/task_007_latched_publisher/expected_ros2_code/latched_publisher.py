#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def main(args=None):
    rclpy.init(args=args)
    node = Node('latched_publisher_node')

    # TODO: fill in durability parameter
    qos = QoSProfile(
        durability=_____,
        reliability=ReliabilityPolicy.RELIABLE,
        depth=1
    )

    pub = node.create_publisher(String, 'latched_topic', qos)
    msg = String()
    msg.data = "Hello, I am latched!"
    pub.publish(msg)
    node.get_logger().info("Message published. Publisher will now exit.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
