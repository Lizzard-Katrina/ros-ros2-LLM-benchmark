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
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('latched_pub_sub_node')

    # TODO: Create a Publisher on 'latched_topic', define a Subscriber with a callback,
    #       and implement the publish loop with rospy.is_shutdown().
    #       The callback should log received messages.
    latched_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )

    pub = node.create_publisher(String, 'latched_topic', latched_qos)

    def msg_callback(msg):
        node.get_logger().info(f'Received: "{msg.data}"')

    sub = node.create_subscription(String, 'latched_topic', msg_callback, latched_qos)

    msg = String()
    count = 0

    while rclpy.ok():
        msg.data = f'Latched message {count}'
        pub.publish(msg)
        count += 1
        rclpy.spin_once(node, timeout_sec=1.0)
    # end: TODO block ends here

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```