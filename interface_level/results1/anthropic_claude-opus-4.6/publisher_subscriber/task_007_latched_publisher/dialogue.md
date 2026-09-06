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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('latched_pub_sub_node')

    # Create a QoS profile with transient local durability (equivalent to ROS1 latched)
    latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    # Create a Publisher on 'latched_topic'
    publisher = node.create_publisher(String, 'latched_topic', latched_qos)

    # Define a callback that logs received messages
    def callback(msg):
        node.get_logger().info('Received: "%s"' % msg.data)

    # Create a Subscriber with the callback
    subscription = node.create_subscription(String, 'latched_topic', callback, latched_qos)

    # Implement the publish loop
    rate = node.create_rate(1)
    count = 0
    while rclpy.ok():
        msg = String()
        msg.data = 'Hello, world! %d' % count
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)
        count += 1
        rclpy.spin_once(node, timeout_sec=0)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```