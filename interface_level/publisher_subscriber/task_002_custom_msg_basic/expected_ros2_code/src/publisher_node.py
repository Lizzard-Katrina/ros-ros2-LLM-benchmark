import rclpy
from rclpy.node import Node
from task_002_custom_msg_basic.msg import Person

class PersonPublisher(Node):
    def __init__(self):
        super().__init__('person_publisher')
        self.publisher_ = self.create_publisher(Person, 'person_info', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Person()
        msg.name = "Alice"
        msg.age = 20
        msg.height = 1.65
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.name}, {msg.age}, {msg.height}')

def main(args=None):
    rclpy.init(args=args)
    node = PersonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
