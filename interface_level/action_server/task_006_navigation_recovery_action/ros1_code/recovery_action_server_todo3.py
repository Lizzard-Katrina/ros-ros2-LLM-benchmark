#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import RecoveryAction, RecoveryResult, RecoveryFeedback

class RecoveryActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'recovery_action',
            RecoveryAction,
            execute_cb=self.execute_cb,
            auto_start=True
        )

    def execute_cb(self, goal):
        # TODO: parse goal to identify recovery type
        # end:   # task ends here
        rospy.loginfo("Parsed recovery goal type: %s", getattr(goal, 'type', 'unknown'))
        for i in range(5):
            feedback = RecoveryFeedback()
            feedback.progress = (i + 1) / 5.0
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = RecoveryResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('recovery_action_server')
    server = RecoveryActionServer()
    rospy.spin()
