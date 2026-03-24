#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.callback,
            10)

    def callback(self, data):
        self.get_logger().info("I heard '%s'" % data.data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        listener.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()