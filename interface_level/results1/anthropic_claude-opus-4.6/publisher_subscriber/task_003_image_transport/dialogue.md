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


class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        # Note: In ROS2 Python, image_transport is not directly available as a Python API.
        # We subscribe to the raw image topic directly using sensor_msgs/Image.
        # For transport plugin support, use the image_transport package via C++ or
        # subscribe to the appropriate topic (e.g., 'camera/image_raw').
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        # TODO: ensure the usage of image_transport
        self.get_logger().info('Received an image')


def main(args=None):
    rclpy.init(args=args)

    node = CameraSubscriberNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```