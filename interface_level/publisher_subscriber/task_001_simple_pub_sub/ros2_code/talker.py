#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = "Hello world %s" % self.get_clock().now().to_msg()
        self.get_logger().info(msg.data)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()