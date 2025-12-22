#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander

class MotionPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.Subscriber("/lego_pose", PoseStamped, self.plan_callback)

    def plan_callback(self, pose):
        # TODO: plan motion to target pose
        # END
