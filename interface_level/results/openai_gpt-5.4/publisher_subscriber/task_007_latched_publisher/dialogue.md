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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class LatchedPubSubNode(Node):
    def __init__(self):
        super().__init__('latched_pub_sub_node')

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(String, 'latched_topic', qos)
        self.subscription = self.create_subscription(
            String,
            'latched_topic',
            self.callback,
            qos,
        )
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS2 {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1


def main():
    rclpy.init()
    node = LatchedPubSubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()