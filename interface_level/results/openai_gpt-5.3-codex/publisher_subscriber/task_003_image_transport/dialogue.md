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

#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Image

try:
    from image_transport import ImageTransport
except ImportError:
    from image_transport_py import ImageTransport

_node = None


def callback(msg):
    # image_transport-based callback
    _node.get_logger().info("Received an image")


def main():
    global _node
    rclpy.init()
    _node = rclpy.create_node('camera_subscriber_node')

    # use image_transport to construct subscriber
    image_transport = ImageTransport(_node)
    sub = image_transport.subscribe('camera/image', callback, 'raw')

    rclpy.spin(_node)

    _node.destroy_node()
    rclpy.shutdown()
    # END OF TODO


if __name__ == '__main__':
    main()
