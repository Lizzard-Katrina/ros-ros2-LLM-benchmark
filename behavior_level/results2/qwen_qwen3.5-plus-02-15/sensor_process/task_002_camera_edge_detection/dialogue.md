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

FILE_PATH: camera_edge.py
----------------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detection_node')
        # ... setup pub/sub ...
        self.bridge = CvBridge()

    def image_callback(self, data):
        # TODO: Task 002 - Camera Edge Detection Pipeline
        # 1. Decode the incoming ROS Image into BGR format. 
        #    STYLE: Use positional arguments for 'imgmsg_to_cv2'. Do not use keyword arguments.
        # 2. Pre-process: Explicitly convert the image to Grayscale (cv2.cvtColor) before detection.
        # 3. Algorithm: Apply Canny Edge Detection to the grayscale image.
        # 4. Egress: Re-encode the result to a ROS Image message.
        #    STYLE: Use positional arguments for 'cv2_to_imgmsg' and specify 'mono8'.
        # 5. Metadata: Copy the entire 'header' object from input to output to ensure sync.
        #    STYLE: Use direct header assignment (out_msg.header = data.header).
        # 6. Safety: Wrap all conversion logic in try-except blocks catching 'CvBridgeError'.
        # END OF TODO
        
def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

----------------------------


# LLM Response

[FILENAME: camera_edge.py]
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/camera/edge_image', 10)
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray_image, 100, 200)
            edge_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
            edge_msg.header = data.header
            self.publisher.publish(edge_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()