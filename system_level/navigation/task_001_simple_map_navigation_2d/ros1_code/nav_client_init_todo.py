#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction

class NavClientInit(object):
    def __init__(self):
        rospy.init_node('nav_client_init')

        self.client = None

        # TODO: initialize move_base action client and wait for server
        # --------------------------
        # The client should connect to the 'move_base' action server
        # and block until the server is available.
        # END

if __name__ == "__main__":
    NavClientInit()
    rospy.spin()
