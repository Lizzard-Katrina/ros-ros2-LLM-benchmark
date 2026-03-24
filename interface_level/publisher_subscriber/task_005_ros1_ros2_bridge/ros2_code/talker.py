#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringPub(Node):
    def __init__(self):
        super().__init__('string_pub')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = "hello from ros2"
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    string_pub = StringPub()
    try:
        rclpy.spin(string_pub)
    except KeyboardInterrupt:
        string_pub.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        string_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()