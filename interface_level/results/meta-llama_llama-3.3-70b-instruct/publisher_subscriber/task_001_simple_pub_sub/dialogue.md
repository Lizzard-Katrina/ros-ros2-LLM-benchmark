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

```python
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def callback(data):
    # Keep logging logic
    print("I heard %s" % data.data)

def listener():
    # TODO: initialize node 'listener'
    rclpy.init()
    node = Node('listener')
    # and subscribe to topic 'chatter'
    node.create_subscription(String, 'chatter', callback)
    # and keep spin
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # END OF TODO
if __name__ == '__main__':
    listener()
```