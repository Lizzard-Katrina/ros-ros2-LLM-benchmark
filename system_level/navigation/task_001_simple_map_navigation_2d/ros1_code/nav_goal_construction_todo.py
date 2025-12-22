#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseGoal
import tf

class NavGoalConstruction(object):
    def __init__(self):
        rospy.init_node('nav_goal_construction')

        self.goal = MoveBaseGoal()

        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # TODO: construct a valid 2D navigation goal (x, y, yaw)
        # --------------------------
        # Fill in position and orientation of the goal.
        # END

if __name__ == "__main__":
    NavGoalConstruction()
    rospy.spin()
