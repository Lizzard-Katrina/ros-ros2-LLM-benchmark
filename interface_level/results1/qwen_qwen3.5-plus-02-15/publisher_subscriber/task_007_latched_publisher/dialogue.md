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
from rclpy.qos import QoSProfile, DurabilityPolicy

def main():
    rclpy.init()
    node = Node('latched_pub_sub_node')

    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def callback(msg):
        node.get_logger().info(f'Received: {msg.data}')

    publisher = node.create_publisher(String, 'latched_topic', qos)
    subscription = node.create_subscription(String, 'latched_topic', callback, qos)

    rate = node.create_rate(1.0)
    try:
        while rclpy.ok():
            msg = String()
            msg.data = "Hello"
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0)
            rate.sleep()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()