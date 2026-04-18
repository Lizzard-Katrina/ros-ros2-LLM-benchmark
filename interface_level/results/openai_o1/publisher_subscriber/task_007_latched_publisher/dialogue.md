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
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('latched_pub_sub_node')

    # TODO: Create a Publisher on 'latched_topic', define a Subscriber with a callback,
    #       and implement the publish loop with rospy.is_shutdown().
    #       The callback should log received messages.
    def callback(msg):
        node.get_logger().info('Received: "%s"' % msg.data)

    publisher = node.create_publisher(String, 'latched_topic', 10)
    subscription = node.create_subscription(String, 'latched_topic', callback, 10)
    msg = String()

    import time
    while rclpy.ok():
        msg.data = "Hello from latched_pub_sub_node!"
        publisher.publish(msg)
        node.get_logger().info('Publishing: "%s"' % msg.data)
        rclpy.spin_once(node)
        time.sleep(1.0)
    # end: TODO block ends here

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()