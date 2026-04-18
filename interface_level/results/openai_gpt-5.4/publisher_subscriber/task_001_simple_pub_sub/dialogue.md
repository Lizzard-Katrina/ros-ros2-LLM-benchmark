# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    # Keep logging logic
    rospy.loginfo("I heard %s", data.data)

def listener():
    # TODO: initialize node 'listener'
    # and subscribe to topic 'chatter'
    # and keep spin
    # END OF TODO
if __name__ == '__main__':
    listener()



----------------------------


# LLM Response

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def callback(data):
    # Keep logging logic
    rclpy.logging.get_logger('listener').info("I heard %s" % data.data)


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            callback,
            10
        )
        self.subscription


def listener():
    # TODO: initialize node 'listener'
    # and subscribe to topic 'chatter'
    # and keep spin
    # END OF TODO
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    listener()