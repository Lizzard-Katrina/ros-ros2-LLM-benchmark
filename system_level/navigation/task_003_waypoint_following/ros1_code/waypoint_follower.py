#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from TODO_fill_goal import fill_goal_from_waypoint

# Waypoint list (x, y, yaw)
WAYPOINTS = [
    (1.0, 0.0, 0.0),
    (2.0, 1.5, 1.57),
    (3.0, 0.0, 3.14)
]

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower_node')
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

    def run(self):
        for idx, wp in enumerate(WAYPOINTS):
            rospy.loginfo(f"Sending waypoint {idx+1}/{len(WAYPOINTS)}: {wp}")

            # --------------------------
            # TODO: MoveBaseGoal
            # END

            self.client.send_goal(goal)
            rospy.loginfo("Goal sent, waiting for result...")
            self.client.wait_for_result()
            state = self.client.get_state()

            # --------------------------
            # TODO:deal with goals
            # END

if __name__ == "__main__":
    follower = WaypointFollower()
    follower.run()
