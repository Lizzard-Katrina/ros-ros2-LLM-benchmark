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
        From navigation/scripts/explore.py:

        for i, r in enumerate(scan.ranges):
            if r > scan.range_min and r < scan.range_max:
                angle = scan.angle_min + i * scan.angle_increment
                x = r * cos(angle)
                y = r * sin(angle)
                frontiers.append((x, y))

        if frontiers:
            goal = random.choice(frontiers)
            self.send_goal(goal)

    def send_goal(self, point):

        goal = MoveBaseGoal()

        # TODO: Populate MoveBaseGoal and send it
        # END


if __name__ == "__main__":
    node = ExplorerNode()
    rospy.spin()
