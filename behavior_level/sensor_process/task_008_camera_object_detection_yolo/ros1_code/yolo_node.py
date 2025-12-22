#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class YoloDetectorNode:
    def __init__(self):
        # TODO: subscribe to /camera/image_raw and publish /detections
        # END: yolo detector setup

    def process_frame(self, img):
        rospy.loginfo("Processing frame for YOLO detection...")

if __name__ == "__main__":
    rospy.init_node("yolo_detector_node")
    node = YoloDetectorNode()
    rospy.spin()
