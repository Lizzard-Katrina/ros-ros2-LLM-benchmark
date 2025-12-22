
import rospy
import smach

class MoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.position = 0.0

    def execute(self, userdata):
        # TODO: implement core arm movement logic
        # Use original variable: self.position
        # Ensure proper state transition
        # END

class CheckReward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.reward = 0

    def execute(self, userdata):
        print("Checking reward...")
        if self.reward > 0:
            return 'success'
        else:
            return 'fail'
