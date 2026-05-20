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

FILE_PATH: camera_publisher.py
----------------------------
#!/usr/bin/env python3
import rospy
# mock Image class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

def main():
    rospy.init_node('camera_publisher_node')
    
    # TODO: use image_transport to construct publisher
    # and insert information of Image

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    # END OF TODO
if __name__ == '__main__':
    main()

----------------------------

FILE_PATH: camera_subscriber.py
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

[FILENAME: camera_publisher.py]
#!/usr/bin/env python3
import rclpy

# mock Image class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_publisher_node')
    
    # TODO: use image_transport to construct publisher
    # and insert information of Image
    pub = node.create_publisher(Image, 'camera/image', 10)

    rate = node.create_rate(10)
    while rclpy.ok():
        msg = Image()
        pub.publish(msg)
        rclpy.spin_once(node)
        rate.sleep()
    # END OF TODO
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

[FILENAME: camera_subscriber.py]
#!/usr/bin/env python3
import rclpy

#mock Image Class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''

node = None

def callback(msg):
    # TODO: ensure the usage of image_transport
    global node
    if node is not None:
        node.get_logger().info("Received an image")

def main(args=None):
    global node
    rclpy.init(args=args)
    node = rclpy.create_node('camera_subscriber_node')
    
    # use image_transport to construct subscriber
    sub = node.create_subscription(Image, 'camera/image', callback, 10)

    rclpy.spin(node)
    # END OF TODO
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()