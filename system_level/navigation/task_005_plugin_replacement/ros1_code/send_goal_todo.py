#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

def send_navigation_goal(x, y, yaw):
    """
    Send a MoveBaseGoal to the move_base action server after planner replacement
    """
    rospy.init_node('plugin_replacement_nav')
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    # TODO: Fill goal pose and header
    # END

    client.send_goal(goal)
    client.wait_for_result()
    state = client.get_state()
    return state == GoalStatus.SUCCEEDED
