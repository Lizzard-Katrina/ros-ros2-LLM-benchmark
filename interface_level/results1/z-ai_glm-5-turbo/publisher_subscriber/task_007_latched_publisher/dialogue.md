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
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = Node('latched_pub_sub_node')

    qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, 'latched_topic', qos_profile)

    def callback(msg):
        node.get_logger().info('Received: "%s"' % msg.data)

    sub = node.create_subscription(String, 'latched_topic', callback, qos_profile)

    while rclpy.ok():
        msg = String()
        msg.data = 'Hello ROS2'
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()