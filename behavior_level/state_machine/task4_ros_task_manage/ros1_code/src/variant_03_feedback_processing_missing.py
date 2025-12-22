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
        goal = MoveGoal()
        goal.position = self.position
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'done'

class TaskFeedback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.feedback_value = 0

    def execute(self, userdata):
        # TODO: process feedback and update self.feedback_value
        # END
