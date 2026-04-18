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

```python
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MockString:
    def __init__(self):
        self.data = ""

def main():
    # TODO: initialize ROS2 node
    rclpy.init()
    node = Node('talker')
    # create publisher for /chatter topic
    pub = node.create_publisher(String, 'chatter')
    # and then publish message
    rate = node.create_rate(1)
    while rclpy.ok():
        msg = String()
        msg.data = "hello from ros2"
        pub.publish(msg)
        node.get_logger().info('Publishing: %s' % msg.data)
        rate.sleep()
    # END OF TODO
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```