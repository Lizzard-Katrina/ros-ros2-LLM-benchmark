#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult, MoveBaseFeedback

class MoveBaseActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'move_base',
            MoveBaseAction,
            execute_cb=self.execute_cb,
            auto_start=True
        )

    def execute_cb(self, goal):
        rospy.loginfo("Received move_base goal")
        # TODO: execute navigation (move robot, check obstacles)
        # end:   # task ends here
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
