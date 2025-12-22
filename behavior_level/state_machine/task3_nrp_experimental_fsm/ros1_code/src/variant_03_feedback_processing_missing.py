import rospy
import smach

class MoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.position = 0.0

    def execute(self, userdata):
        print(f"Moving arm to position {self.position}")
        self.position += 1.0
        return 'done'

class CheckReward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.reward = 0

    def execute(self, userdata):
        # TODO: process feedback from simulation
        # Update self.reward according to original logic
        # END
