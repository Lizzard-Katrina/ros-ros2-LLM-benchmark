import rospy
import smach

class MoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.position = 0.0

    def execute(self, userdata):
        # TODO: Send command to NRP simulation environment
        # Use self.position
        # END

import rospy
import smach

class MoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.position = 0.0

    def execute(self, userdata):
        # TODO: Send command to NRP simulation environment
        # Use self.position
        # END

class CheckReward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.reward = 0
        return 'done'

class CheckReward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.reward = 0
