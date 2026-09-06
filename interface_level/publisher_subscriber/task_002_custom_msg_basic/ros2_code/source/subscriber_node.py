#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from task_002_custom_msg_basic.msg import Person


class PersonSubscriber(Node):
    def __init__(self):
        super().__init__('person_subscriber')
        self.subscription = self.create_subscription(
            Person,
            'person_info',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f'Received: name={msg.name}, age={msg.age}, height={msg.height}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()