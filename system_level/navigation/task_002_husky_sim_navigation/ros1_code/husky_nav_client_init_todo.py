#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction

class HuskyNavClientInit(object):
    def __init__(self):
        rospy.init_node('husky_nav_client_init')

        self.client = None

        # TODO: initialize move_base action client for Husky navigation
        # END

if __name__ == "__main__":
    HuskyNavClientInit()
    rospy.spin()
