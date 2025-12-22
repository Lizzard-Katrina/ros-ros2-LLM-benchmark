#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

class DepthSubscriber:
    def __init__(self):
        # TODO: initialize depth image subscriber (subscribe to /camera/depth)
        # END: depth subscriber initialization ends here

    def callback(self, msg):
        rospy.loginfo("Depth Image received")

if __name__ == "__main__":
    rospy.init_node("depth_subscriber_node")
    node = DepthSubscriber()
    rospy.spin()
