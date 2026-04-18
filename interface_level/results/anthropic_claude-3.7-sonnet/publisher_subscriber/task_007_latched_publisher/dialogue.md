# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
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

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class LatchedPubSubNode(Node):
    def __init__(self):
        super().__init__('latched_pub_sub_node')
        
        # Create QoS profile with transient local durability (equivalent to ROS1 latched)
        latched_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # Create publisher with latched QoS
        self.publisher = self.create_publisher(
            String, 
            'latched_topic', 
            qos_profile=latched_qos
        )
        
        # Create subscriber with same QoS
        self.subscription = self.create_subscription(
            String,
            'latched_topic',
            self.callback,
            qos_profile=latched_qos
        )
        
        # Timer for publishing
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0
        
    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        
    def publish_message(self):
        msg = String()
        msg.data = f'Latched message {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1

def main():
    rclpy.init()
    node = LatchedPubSubNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```