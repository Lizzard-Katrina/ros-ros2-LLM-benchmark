import rospy
import smach
import actionlib
from some_robot_msgs.msg import MoveAction, MoveGoal

class MoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fail'])
        self.client = actionlib.SimpleActionClient('move_arm', MoveAction)
        self.client.wait_for_server()
        self.position = 0.0

    def execute(self, userdata):
        # TODO: implement planning logic for state transitions
        # Use self.position and self.client
        # Ensure correct transitions: 'done' or 'fail'
        # END

class TaskFeedback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.feedback_value = 0

    def execute(self, userdata):
        print("Processing feedback...")
        if self.feedback_value > 0:
            return 'success'
        else:
            return 'fail'
