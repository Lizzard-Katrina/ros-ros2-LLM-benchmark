#!/usr/bin/env python
# ORIGINAL: move_base_state__ORIGINAL.py

import rospy
import actionlib
from some_robot_msgs.msg import MoveAction, MoveGoal

class MoveBaseState(object):
    def __init__(self, name='MoveBaseState'):
        self.client = actionlib.SimpleActionClient('move_base', MoveAction)
        self.client.wait_for_server()
        self.name = name
        self.userdata = type('ud', (), {})()
        self.outcomes = ['done', 'fail']

    def execute(self, userdata):
        rospy.loginfo("[MoveBaseState] sending goal")
        goal = MoveGoal()
        goal.target = getattr(userdata, 'target', None)
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(5.0))
        if not finished:
            self.client.cancel_all_goals()
            return 'fail'
        result = self.client.get_result()
        return 'done' if getattr(result, 'success', False) else 'fail'

