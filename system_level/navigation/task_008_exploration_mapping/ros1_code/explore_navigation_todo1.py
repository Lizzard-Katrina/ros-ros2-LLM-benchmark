#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import random
import math


class ExplorerNode:
    def __init__(self):
        rospy.init_node("explorer_node")

        self.client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.client.wait_for_server()

        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.scan_callback
        )

        self.goal_pub = rospy.Publisher(
            "/exploration_goal", PoseStamped, queue_size=1
        )

        self.frontier_points = []

    def scan_callback(self, scan_msg):

        # TODO: Extract frontier points from laser scan
        # END

    def send_goal(self, point):
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)

        goal = MoveBaseGoal()


if __name__ == "__main__":
    node = ExplorerNode()
    rospy.spin()
