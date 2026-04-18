# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python3
import rospy
#mock Image Class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''
def callback(msg):
    # TODO: ensure the usage of image_transport
    rospy.loginfo("Received an image")

def main():
    rospy.init_node('camera_subscriber_node')
    
    # use image_transport to construct subscriber
    # sub = ...

    rospy.spin()
    # END OF TODO
if __name__ == '__main__':
    main()

----------------------------


# LLM Response

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from image_transport import ImageTransport

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        qos_profile = QoSProfile(depth=10)
        image_transport = ImageTransport(self)
        self.subscriber = image_transport.subscribe('camera/image', self.callback, qos_profile=qos_profile)

    def callback(self, msg):
        self.get_logger().info("Received an image")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```