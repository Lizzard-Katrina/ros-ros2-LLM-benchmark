# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python
import rospy

#mock string class
class String:
    def __init__(self):
        self.data = ""


def main():
    # TODO: initialize ROS1 node
    # create publisher for /chatter topic
    # and hen publish message
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "hello from ros1"
        rate.sleep()
    # END OF TODO
if __name__ == "__main__":
    main()

----------------------------


# LLM Response

#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String as ROSString


# mock string class
class String:
    def __init__(self):
        self.data = ""


def main():
    rclpy.init()
    node = rclpy.create_node("chatter_publisher")
    publisher = node.create_publisher(ROSString, "/chatter", 10)

    rate = node.create_rate(1.0)
    while rclpy.ok():
        msg = ROSString()
        msg.data = "hello from ros1"
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()