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
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('latched_pub_sub_node')

    # TODO: Create a Publisher on 'latched_topic', define a Subscriber with a callback,
    #       and implement the publish loop with rospy.is_shutdown().
    #       The callback should log received messages.
    qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )

    def callback(msg: String):
        node.get_logger().info(f"Received: {msg.data}")

    publisher = node.create_publisher(String, 'latched_topic', qos_profile)
    subscriber = node.create_subscription(String, 'latched_topic', callback, qos_profile)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    count = 0
    try:
        while rclpy.ok():
            msg = String()
            msg.data = f"Hello from ROS2 (latched) #{count}"
            publisher.publish(msg)
            node.get_logger().info(f"Published: {msg.data}")
            executor.spin_once(timeout_sec=0.1)
            time.sleep(1.0)
            count += 1
    finally:
        node.destroy_subscription(subscriber)
        node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()
    # end: TODO block ends here


if __name__ == "__main__":
    main()