# TODO: Fill in actionlib call logic
# Original context preserved

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
        # TODO: send MoveGoal using actionlib client
        # Ensure proper goal construction and wait for result
        # END

class TaskFeedback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.feedback_value = 0
