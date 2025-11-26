#!/usr/bin/env python
import rospy
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupResult, MoveGroupFeedback

class MoveItActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'moveit_action',
            MoveGroupAction,
            execute_cb=self.execute_cb,
            auto_start=True
        )

    def execute_cb(self, goal):
        rospy.loginfo("Received MoveIt motion planning goal")
        # TODO: execute motion (plan trajectory, update robot)
        # end:   # task ends here
        for i in range(5):
            feedback = MoveGroupFeedback()
            feedback.state = i
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = MoveGroupResult()
        result.error_code.val = 1
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('moveit_action_server')
    server = MoveItActionServer()
    rospy.spin()
