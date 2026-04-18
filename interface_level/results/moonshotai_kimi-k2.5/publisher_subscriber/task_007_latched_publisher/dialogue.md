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
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('latched_pub_sub_node')

    # TODO: Create a Publisher on 'latched_topic', define a Subscriber with a callback,
    #       and implement the publish loop with rospy.is_shutdown().
    #       The callback should log received messages.
    qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, 'latched_topic', qos_profile)
    
    def callback(msg):
        node.get_logger().info('Received: "%s"' % msg.data)
    
    subscription = node.create_subscription(String, 'latched_topic', callback, qos_profile)
    
    count = 0
    rate = node.create_rate(10)
    
    while rclpy.ok():
        msg = String()
        msg.data = 'Hello World %d' % count
        publisher.publish(msg)
        count += 1
        rclpy.spin_once(node, timeout_sec=0)
        rate.sleep()
    # end: TODO block ends here

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()