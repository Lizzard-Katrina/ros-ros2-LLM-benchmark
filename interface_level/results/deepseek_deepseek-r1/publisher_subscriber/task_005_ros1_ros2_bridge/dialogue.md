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
from std_msgs.msg import String

class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('ros2_publisher')
        self.publisher = self.create_publisher(String, '/chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = "hello from ros2"
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ROS2Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()