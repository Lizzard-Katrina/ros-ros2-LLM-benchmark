import rospy
import smach

class MoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.position = 0.0

    def execute(self, userdata):
        # simulate arm movement
        print(f"Moving arm to position {self.position}")
        self.position += 1.0
        return 'done'

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

def main():
    rospy.init_node('nrp_experiment_fsm')
    sm = smach.StateMachine(outcomes=['FINISHED'])
    with sm:
        smach.StateMachine.add('MOVE_ARM', MoveArm(), transitions={'done':'CHECK_REWARD'})
        smach.StateMachine.add('CHECK_REWARD', CheckReward(), transitions={'success':'FINISHED','fail':'MOVE_ARM'})
    outcome = sm.execute()

if __name__ == "__main__":
    main()
