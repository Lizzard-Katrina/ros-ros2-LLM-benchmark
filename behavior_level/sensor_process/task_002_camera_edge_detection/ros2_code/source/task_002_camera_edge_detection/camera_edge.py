import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detection_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, 'camera/edges', 10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)

        try:
            output_msg = self.bridge.cv2_to_imgmsg(edges, 'mono8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        output_msg.header = data.header
        self.publisher.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()