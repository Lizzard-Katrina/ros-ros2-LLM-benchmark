#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class StereoSubscriber:
    def __init__(self):
        # TODO: subscribe to /left/image_raw and /right/image_raw
        # END: stereo subscriber setup

    def left_callback(self, msg):
        rospy.loginfo("Left image received.")

    def right_callback(self, msg):
        rospy.loginfo("Right image received.")

if __name__ == "__main__":
    rospy.init_node("stereo_subscriber_node")
    node = StereoSubscriber()
    rospy.spin()
