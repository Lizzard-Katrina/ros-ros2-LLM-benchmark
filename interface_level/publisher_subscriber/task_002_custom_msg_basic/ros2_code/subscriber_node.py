#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0

class PersonSubscriber(Node):
    def __init__(self):
        super().__init__('person_subscriber')
        self.subscription = self.create_subscription(
            String,
            'person_info',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        # TODO
        # Print received data
        self.get_logger().info(f"Received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    person_subscriber = PersonSubscriber()
    try:
        rclpy.spin(person_subscriber)
    except KeyboardInterrupt:
        person_subscriber.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        person_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()