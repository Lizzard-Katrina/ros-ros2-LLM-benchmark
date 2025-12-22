#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class DepthProcessor:
    def __init__(self):
        # TODO: subscribe to /left/image_raw and /right/image_raw, then publish /depth/image
        # END: depth processor setup

    def process_stereo(self, left_img, right_img):
        rospy.loginfo("Processing stereo images to produce depth map...")

if __name__ == "__main__":
    rospy.init_node("stereo_depth_processor")
    node = DepthProcessor()
    rospy.spin()
