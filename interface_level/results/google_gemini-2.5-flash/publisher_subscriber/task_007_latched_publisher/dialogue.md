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

class LatchedPublisherSubscriber(Node):
    def __init__(self):
        super().__init__('latched_pub_sub_node')
        # TODO: Create a Publisher on 'latched_topic', define a Subscriber with a callback,
        #       and implement the publish loop with rospy.is_shutdown().
        #       The callback should log received messages.
        self.publisher_ = self.create_publisher(String, 'latched_topic', qos_profile=rclpy.qos.qos_profile_system_default)
        self.subscription = self.create_subscription(
            String,
            'latched_topic',
            self.listener_callback,
            qos_profile=rclpy.qos.qos_profile_system_default)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(1.0, self.publish_message) # Publish every 1 second
        self.i = 0
        # end: TODO block ends here

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello ROS2 Latched World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = LatchedPublisherSubscriber()

    # TODO: Implement the publish loop with rospy.is_shutdown().
    #       This is now handled by rclpy.spin() and the timer in the class.
    rclpy.spin(node)
    # end: TODO block ends here

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```