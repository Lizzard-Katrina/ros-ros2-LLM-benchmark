#!/usr/bin/env python
import rospy
import smach
from std_msgs.msg import String

class LockedState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['unlock'])
        self.coin_received = False
        self.pub = rospy.Publisher('state_info', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Locked state")
        while not self.coin_received:
            # --------------------------
            # TODO: Translate this ROS1 coin check and publishing to ROS2
            # END
            rospy.sleep(0.5)
        return 'unlock'
