#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class HuskyGoalExecution(object):
    def __init__(self):
        rospy.init_node('husky_goal_execution')

        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.orientation.w = 1.0

        # TODO: send goal to Husky and verify navigation result
        # END

if __name__ == "__main__":
    HuskyGoalExecution()
    rospy.spin()
