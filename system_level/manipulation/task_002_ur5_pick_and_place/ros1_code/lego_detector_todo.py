#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class LegoDetector:
    def __init__(self):
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher("/lego_pose", PoseStamped, queue_size=10)

    def image_callback(self, msg):
        # TODO: detect LEGO brick and publish its pose
        # END
