#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server

class DynamicNode:
    def __init__(self):
        rospy.init_node("dynamic_param_node")

        # TODO: Setup dynamic parameter reconfigure callbacks
        # END_TODO

        rospy.loginfo("Dynamic parameter node initialized.")

if __name__ == "__main__":
    node = DynamicNode()
    rospy.spin()
