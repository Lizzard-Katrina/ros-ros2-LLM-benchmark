#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('latched_pub_sub_node')

    latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def callback(msg):
        node.get_logger().info('Received: "%s"' % msg.data)

    pub = node.create_publisher(String, 'latched_topic', latching_qos)
    sub = node.create_subscription(String, 'latched_topic', callback, latching_qos)

    count = 0
    while rclpy.ok():
        msg = String()
        msg.data = 'Hello World %d' % count
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=1.0)
        count += 1

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()