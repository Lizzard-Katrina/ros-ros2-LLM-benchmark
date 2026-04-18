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
import image_transport

# mock Image Class
class Image:
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

def callback(msg):
    # TODO: ensure the usage of image_transport
    print("Received an image")

def main():
    rclpy.init()
    node = rclpy.create_node('camera_subscriber_node')

    # use image_transport to construct subscriber
    # sub = ...
    sub = image_transport.ImageTransport(node).subscribe('/camera/image_raw', callback, 'raw')

    rclpy.spin(node)
    # END OF TODO
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()