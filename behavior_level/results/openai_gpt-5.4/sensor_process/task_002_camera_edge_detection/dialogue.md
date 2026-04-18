# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detection_node')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, 'image_edges', 10)
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10
        )

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray_image, 100, 200)
            out_msg = self.bridge.cv2_to_imgmsg(edges, 'mono8')
            out_msg.header = data.header
            self.publisher_.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()