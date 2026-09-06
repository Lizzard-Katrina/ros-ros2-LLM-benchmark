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
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
#mock Image Class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

g_node = None

def callback(msg):
    # TODO: ensure the usage of image_transport
    g_node.get_logger().info("Received an image")

def main():
    global g_node
    rclpy.init()
    g_node = Node('camera_subscriber_node')
    
    # use image_transport to construct subscriber
    # sub = ...
    sub = g_node.create_subscription(SensorImage, 'image_topic', callback, 10)

    rclpy.spin(g_node)
    # END OF TODO
    g_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()