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

```python
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class LatchedPubSubNode(Node):
    def __init__(self):
        super().__init__('latched_pub_sub_node')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(String, 'latched_topic', qos_profile)
        self.sub = self.create_subscription(String, 'latched_topic', self.callback, qos_profile)

        self.get_logger().info('Latched pub/sub node initialized')

    def callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

    def publish_loop(self):
        rate = self.create_rate(1)
        msg = String()
        while rclpy.ok():
            msg.data = f'Hello ROS2 at {self.get_clock().now().to_msg().sec}'
            self.pub.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            rate.sleep()

def main():
    rclpy.init()
    node = LatchedPubSubNode()
    node.publish_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```