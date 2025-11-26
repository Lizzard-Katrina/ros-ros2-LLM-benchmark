#!/usr/bin/env python
import rospy
import actionlib
from moveit_tutorials.msg import MoveArmCartesianAction, MoveArmCartesianResult, MoveArmCartesianFeedback

class MoveArmCartesianActionServer:
    def __init__(self):
        # TODO: create SimpleActionServer instance
        # end:   # task ends here
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received Cartesian goal")
        for i in range(5):
            feedback = MoveArmCartesianFeedback()
            feedback.pose = [0.0, 0.0, 0.0]  # dummy Cartesian pose
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = MoveArmCartesianResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('move_arm_cartesian_action_server')
    server = MoveArmCartesianActionServer()
    rospy.spin()
