#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class RGBSubscriber:
    def __init__(self):
        # TODO: initialize RGB image subscriber (subscribe to /camera/rgb/image_color)
        # END: RGB subscriber initialization ends here

    def callback(self, msg):
        rospy.loginfo("RGB Image received")

if __name__ == "__main__":
    rospy.init_node("rgb_subscriber_node")
    node = RGBSubscriber()
    rospy.spin()
