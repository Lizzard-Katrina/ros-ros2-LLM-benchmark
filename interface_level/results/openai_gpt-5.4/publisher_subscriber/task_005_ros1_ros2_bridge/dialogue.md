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
from rclpy.node import Node

#mock string class
class String:
    def __init__(self):
        self.data = ""


class ChatterNode(Node):
    def __init__(self):
        super().__init__('chatter_node')
        self.publisher_ = self.create_publisher(String, '/chatter', 10)


def main():
    # TODO: initialize ROS1 node
    # create publisher for /chatter topic
    # and hen publish message
    rclpy.init()
    node = ChatterNode()
    rate = node.create_rate(1)
    while rclpy.ok():
        msg = String()
        msg.data = "hello from ros1"
        node.publisher_.publish(msg)
        rate.sleep()
    node.destroy_node()
    rclpy.shutdown()
    # END OF TODO
if __name__ == "__main__":
    main()