#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from task_002_custom_msg_basic.msg import Person


class PersonPublisher(Node):
    def __init__(self):
        super().__init__('person_publisher')
        self.publisher_ = self.create_publisher(Person, 'person_info', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Person()
        msg.name = 'Alice'
        msg.age = 30
        msg.height = 1.65
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: name={msg.name}, age={msg.age}, height={msg.height}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()