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
from std_msgs.msg import String
import json


class PersonPublisher(Node):
    def __init__(self):
        super().__init__('person_publisher')
        self.publisher_ = self.create_publisher(String, '/person_info', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        person = {
            'name': 'Alice',
            'age': 30,
            'height': 165
        }
        msg.data = json.dumps(person)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: name={person["name"]}, age={person["age"]}, height={person["height"]}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

[FILENAME: subscriber_node.py]
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class PersonSubscriber(Node):
    def __init__(self):
        super().__init__('person_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/person_info',
            self.callback,
            10
        )

    def callback(self, msg):
        person = json.loads(msg.data)
        self.get_logger().info(
            f"Received: name={person['name']}, age={person['age']}, height={person['height']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PersonSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()