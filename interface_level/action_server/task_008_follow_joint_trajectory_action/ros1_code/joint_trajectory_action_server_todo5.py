#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback

class JointTrajectoryActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=True
        )

    def execute_cb(self, goal):
        rospy.loginfo("Received joint trajectory goal")
        for i in range(5):
            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = goal.trajectory.joint_names
            feedback.actual.positions = [0.1*i for _ in goal.trajectory.joint_names]
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = FollowJointTrajectoryResult()
        # TODO: set result.error_code based on execution outcome
        # end:   # task ends here
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('joint_trajectory_action_server')
    server = JointTrajectoryActionServer()
    rospy.spin()
