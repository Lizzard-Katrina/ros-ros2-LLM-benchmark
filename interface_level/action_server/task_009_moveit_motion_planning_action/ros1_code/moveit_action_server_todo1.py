#!/usr/bin/env python
import rospy
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupResult, MoveGroupFeedback

class MoveItActionServer:
    def __init__(self):
        # TODO: create SimpleActionServer instance
        # end:   # task ends here
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received MoveIt motion planning goal")
        for i in range(5):
            feedback = MoveGroupFeedback()
            feedback.state = i  # dummy feedback
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = MoveGroupResult()
        result.error_code.val = 1  # success
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('moveit_action_server')
    server = MoveItActionServer()
    rospy.spin()
