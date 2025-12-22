#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class EdgeDetector:
    def __init__(self):
        rospy.init_node("edge_detector", anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/edges", Image, queue_size=10)
        rospy.Subscriber("/camera/image_raw", Image, self.callback)
        rospy.loginfo("Edge detector node started")

    def callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Apply Canny edge detection
        edges = cv2.Canny(cv_image, 100, 200)

        # Convert back to ROS Image and publish
        ros_edges = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")
        self.pub.publish(ros_edges)

if __name__ == "__main__":
    node = EdgeDetector()
    rospy.spin()
