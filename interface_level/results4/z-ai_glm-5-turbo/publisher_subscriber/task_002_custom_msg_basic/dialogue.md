# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: publisher_node.py
----------------------------
#!/usr/bin/env python3
import rospy

# Mock of ROS1 custom message Person
class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0


def main():
    rospy.init_node('person_publisher')

    # ======= STUDENT TODO ========
    # Create a publisher named /person_info
    # publishing the custom Person message.
    # Fill the message fields and publish at 1 Hz.


    while not rospy.is_shutdown():
        msg = Person()

        # Fill the message fields: name, age, height
        # =============================
        rate.sleep()
        # END OF TODO
if __name__ == '__main__':
    main()

----------------------------

FILE_PATH: subscriber_node.py
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

[FILENAME: publisher_node.py]
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Mock of ROS1 custom message Person
class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0


def main():
    rclpy.init()
    node = Node('person_publisher')

    # ======= STUDENT TODO ========
    # Create a publisher named /person_info
    # publishing the custom Person message.
    # Fill the message fields and publish at 1 Hz.
    pub = node.create_publisher(Person, '/person_info', 10)
    rate = node.create_rate(1.0)

    while rclpy.ok():
        msg = Person()

        # Fill the message fields: name, age, height
        msg.name = "John Doe"
        msg.age = 30
        msg.height = 1.75
        pub.publish(msg)
        # =============================
        rate.sleep()
        # END OF TODO
if __name__ == '__main__':
    main()

[FILENAME: subscriber_node.py]
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0


def callback(msg):
    # TODO
    # Print received data
    node.get_logger().info(f"Received: {msg}")

def main():
    rclpy.init()
    global node
    node = Node('person_subscriber')

    # Create a subscriber listening to /person_info
    sub = node.create_subscription(Person, '/person_info', callback, 10)

    rclpy.spin(node)
    #END OF TODO
if __name__ == '__main__':
    main()