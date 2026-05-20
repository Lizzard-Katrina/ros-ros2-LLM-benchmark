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

FILE_PATH: latched_publisher.py
----------------------------
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('latched_pub_sub_node')

    # TODO: Create a Publisher on 'latched_topic', define a Subscriber with a callback,
    #       and implement the publish loop with rospy.is_shutdown().
    #       The callback should log received messages.
    # end: TODO block ends here

if __name__ == "__main__":
    main()

----------------------------


# LLM Response

[FILENAME: latched_publisher.py]
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LatchedPubSubNode(Node):
    def __init__(self):
        super().__init__('latched_pub_sub_node')
        
        # Create a Publisher on 'latched_topic' with latch behavior (QoS Durability TRANSIENT_LOCAL)
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.publisher_ = self.create_publisher(String, 'latched_topic', qos_profile)
        
        # Define a Subscriber with a callback
        self.subscription = self.create_subscription(
            String,
            'latched_topic',
            self.callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def publish_loop(self):
        msg = String()
        msg.data = "Hello from ROS2 Latched Publisher"
        
        rate = self.create_rate(1.0)
        
        while rclpy.ok():
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            rate.sleep()

def main():
    rclpy.init()
    node = LatchedPubSubNode()
    
    try:
        node.publish_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()