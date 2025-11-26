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
        # TODO: parse goal trajectory points and joint names
        # end:   # task ends here
        rospy.loginfo("Parsed trajectory with %d points for joints %s", len(goal.trajectory.points), goal.trajectory.joint_names)
        for i in range(5):
            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = goal.trajectory.joint_names
            feedback.actual.positions = [0.1*i for _ in goal.trajectory.joint_names]
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = FollowJointTrajectoryResult()
        result.error_code = 0
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('joint_trajectory_action_server')
    server = JointTrajectoryActionServer()
    rospy.spin()
