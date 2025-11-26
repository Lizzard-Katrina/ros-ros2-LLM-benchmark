#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult, MoveBaseFeedback

class MoveBaseActionServer:
    def __init__(self):
        # TODO: create SimpleActionServer instance
        # end:   # task ends here
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received move_base goal")
        for i in range(5):
            feedback = MoveBaseFeedback()
            feedback.base_position = i * 0.2
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = MoveBaseResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('move_base_action_server')
    server = MoveBaseActionServer()
    rospy.spin()
