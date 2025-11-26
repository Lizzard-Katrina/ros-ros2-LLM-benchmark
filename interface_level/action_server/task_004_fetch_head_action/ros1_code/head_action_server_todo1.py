#!/usr/bin/env python
import rospy
import actionlib
from fetch_head_msgs.msg import HeadPointingAction, HeadPointingResult, HeadPointingFeedback

class HeadActionServer:
    def __init__(self):
        # TODO: create SimpleActionServer instance
        # end:   # task ends here
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received target TF: %s", goal.target_frame)
        for i in range(5):
            feedback = HeadPointingFeedback()
            feedback.progress = (i + 1) / 5.0
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = HeadPointingResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('head_action_server')
    server = HeadActionServer()
    rospy.spin()
