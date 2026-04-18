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
from image_transport import ImageTransport

#mock Image Class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

def callback(msg):
    # TODO: ensure the usage of image_transport
    rclpy.logging.get_logger('camera_subscriber_node').info("Received an image")

def main():
    rclpy.init()
    node = rclpy.create_node('camera_subscriber_node')
    
    # use image_transport to construct subscriber
    it = ImageTransport(node)
    sub = it.subscribe('camera/image', callback, 10)
    
    rclpy.spin(node)
    # END OF TODO
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()