#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class CameraSubscriber:
    def __init__(self):
        # TODO: subscribe to /camera/image_raw and store the latest frame
        # END: camera subscriber setup

    def callback(self, msg):
        rospy.loginfo("Camera frame received.")

if __name__ == "__main__":
    rospy.init_node("camera_subscriber_node")
    node = CameraSubscriber()
    rospy.spin()
