#!/usr/bin/env python
"""
Original source:
UR5-Pick-and-Place-Simulation/ur5_control/scripts/pick_and_place.py
"""

import rospy
from std_msgs.msg import Bool

class PickAndPlace:
    def __init__(self):
        rospy.Subscriber("/motion_done", Bool, self.execute)

    def execute(self, msg):
        # TODO: control gripper and place object
        # END
