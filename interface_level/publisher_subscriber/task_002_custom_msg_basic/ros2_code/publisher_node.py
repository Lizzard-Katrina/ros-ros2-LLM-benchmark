#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from example_interfaces.msg import Int64
from custom_msgs.msg import Person  # Assuming you've created a ROS2 custom message package

class PersonPublisher(Node):
    def __init__(self):
        super().__init__('person_publisher')
        self.publisher_ = self.create_publisher(Person, 'person_info', 10)
        self.timer = self.create_timer(1.0, self.publish_person_info)

    def publish_person_info(self):
        msg = Person()
        msg.name = "John Doe"
        msg.age = 30
        msg.height = 175
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing person info: "%s", %d, %d' % (msg.name, msg.age, msg.height))

def main(args=None):
    rclpy.init(args=args)
    person_publisher = PersonPublisher()
    try:
        rclpy.spin(person_publisher)
    except KeyboardInterrupt:
        person_publisher.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        person_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()