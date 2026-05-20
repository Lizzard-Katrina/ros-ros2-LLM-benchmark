#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

def talker():
    rclpy.init()
    node = rclpy.create_node('talker')
    pub = node.create_publisher(String, 'chatter', 10)
    
    rate = node.create_rate(1)  # Keep this line

    while rclpy.ok():
        msg_str = "Hello world %s" % (node.get_clock().now().nanoseconds / 1e9)   # Keep message logic
        node.get_logger().info(msg_str)

        msg = String()
        msg.data = msg_str
        pub.publish(msg)
        
        rclpy.spin_once(node)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        pass