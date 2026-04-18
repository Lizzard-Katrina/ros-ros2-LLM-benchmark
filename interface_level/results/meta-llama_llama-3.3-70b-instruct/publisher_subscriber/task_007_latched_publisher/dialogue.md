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
from std_msgs.msg import String

class LatchedPubSubNode(Node):
    def __init__(self):
        super().__init__('latched_pub_sub_node')
        self.publisher = self.create_publisher(String, 'latched_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'latched_topic',
            self.listener_callback,
            10)
        self.timer = self.create_timer(0.5, self.publish_loop)

    def listener_callback(self, msg):
        self.get_logger().info('Received message: %s' % msg.data)

    def publish_loop(self):
        if not rclpy.ok():
            return
        msg = String()
        msg.data = 'Hello, world!'
        self.publisher.publish(msg)
        self.get_logger().info('Published message: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = LatchedPubSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```