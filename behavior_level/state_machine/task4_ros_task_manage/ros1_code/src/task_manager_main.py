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
        print(f"Sending arm to {self.position}")
        goal = MoveGoal()
        goal.position = self.position
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        if result.success:
            return 'done'
        else:
            return 'fail'

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

def main():
    rospy.init_node('ros_task_manager')
    sm = smach.StateMachine(outcomes=['FINISHED'])
    with sm:
        smach.StateMachine.add('MOVE_ARM', MoveArm(), transitions={'done':'TASK_FEEDBACK','fail':'MOVE_ARM'})
        smach.StateMachine.add('TASK_FEEDBACK', TaskFeedback(), transitions={'success':'FINISHED','fail':'MOVE_ARM'})
    outcome = sm.execute()

if __name__ == "__main__":
    main()
