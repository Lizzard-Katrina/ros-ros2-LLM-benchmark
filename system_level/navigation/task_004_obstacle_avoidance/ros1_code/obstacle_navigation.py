#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from TODO_fill_navigation_goal import fill_navigation_goal

class Turtlebot3ObstacleNav:
    def __init__(self):
        rospy.init_node('turtlebot3_obstacle_navigation')
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server…")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

    def run(self):
        # Example goal list
        goals = [
            (1.0, 0.0, 0.0),
            (2.0, -1.0, 1.57),
            (0.0, -2.0, 3.14)
        ]

        for idx, g in enumerate(goals):
            rospy.loginfo(f"Sending goal {idx+1}/{len(goals)}: {g}")

            # TODO: fill the navigation goal for turtlebot3
            # END

            self.client.send_goal(goal)
            self.client.wait_for_result()
            state = self.client.get_state()

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Goal {idx+1} succeeded!")
            else:
                rospy.logwarn(f"Goal {idx+1} failed with state {state}")

if __name__ == "__main__":
    nav = Turtlebot3ObstacleNav()
    nav.run()
