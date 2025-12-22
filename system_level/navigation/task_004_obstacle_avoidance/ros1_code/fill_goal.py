#!/usr/bin/env python
import tf
from move_base_msgs.msg import MoveBaseGoal

def fill_navigation_goal(goal_point):
    """
    Convert goal_point (x, y, yaw) into a MoveBaseGoal.
    The navigation stack (including obstacle avoidance) will handle path planning.
    """
    goal = MoveBaseGoal()
    # TODO: Construct navigation goal for TurtleBot3
    # END
    return goal
