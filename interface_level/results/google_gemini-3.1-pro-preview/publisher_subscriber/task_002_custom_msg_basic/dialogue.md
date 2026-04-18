# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python3
import rospy
class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0


def callback(msg):
    # TODO
    # Print received data
    rospy.loginfo(f"Received: {msg}")

def main():
    rospy.init_node('person_subscriber')

    # Create a subscriber listening to /person_info

    rospy.spin()
    #END OF TODO
if __name__ == '__main__':
    main()

----------------------------


# LLM Response

```python
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0

def callback(msg):
    # TODO
    # Print received data
    rclpy.logging.get_logger('person_subscriber').info(f"Received: {msg}")

def main():
    rclpy.init()
    node = rclpy.create_node('person_subscriber')

    # Create a subscriber listening to /person_info
    node.create_subscription(String, '/person_info', callback, 10)

    rclpy.spin(node)
    #END OF TODO
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```