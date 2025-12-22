#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseGoal
import tf

class HuskyGoalConstruction(object):
    def __init__(self):
        rospy.init_node('husky_goal_construction')

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # TODO: construct a valid 2D navigation goal for Husky in simulation
        # END

if __name__ == "__main__":
    HuskyGoalConstruction()
    rospy.spin()
