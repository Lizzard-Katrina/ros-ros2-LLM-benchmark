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
from rclpy.node import Node
# TODO: Import the correct message type
from std_msgs.msg import String # Assuming a simple string message for demonstration

class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0

# TODO: Create a ROS2 Node class
class PersonSubscriber(Node):
    def __init__(self):
        super().__init__('person_subscriber')
        # TODO: Create a subscriber listening to /person_info
        self.subscription = self.create_subscription(
            String,  # TODO: Replace with the actual message type for Person info
            'person_info',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        # TODO: Print received data
        self.get_logger().info(f"Received: {msg.data}") # Assuming msg.data for a simple message


def main(args=None):
    rclpy.init(args=args)
    
    person_subscriber = PersonSubscriber()

    rclpy.spin(person_subscriber)

    person_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```