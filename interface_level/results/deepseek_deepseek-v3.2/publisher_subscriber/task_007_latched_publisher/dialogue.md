# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('latched_pub_sub_node')

    # TODO: Create a Publisher on 'latched_topic', define a Subscriber with a callback,
    #       and implement the publish loop with rospy.is_shutdown().
    #       The callback should log received messages.
    # end: TODO block ends here

if __name__ == "__main__":
    main()

----------------------------


# LLM Response

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LatchedPubSubNode(Node):
    def __init__(self):
        super().__init__('latched_pub_sub_node')
        self.publisher_ = self.create_publisher(String, 'latched_topic', 10)
        self.subscription_ = self.create_subscription(
            String,
            'latched_topic',
            self.listener_callback,
            10
        )
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.count_ = 0

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count_}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count_ += 1

def main():
    rclpy.init()
    node = LatchedPubSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()