#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class NavGoalExecution(object):
    def __init__(self):
        rospy.init_node('nav_goal_execution')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # TODO: send navigation goal and handle execution result
        # END

if __name__ == "__main__":
    NavGoalExecution()
    rospy.spin()
