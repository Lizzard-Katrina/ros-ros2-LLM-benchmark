#!/usr/bin/env python
# ORIGINAL: action_client_wrapper.py

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from mas_exec_manager.msg import TaskRequestAction, TaskRequestGoal

class ActionExecutor(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/task_request", TaskRequestAction)
        rospy.loginfo("[ActionExecutor] Waiting for action server /task_request ...")
        self.client.wait_for_server()
        rospy.loginfo("[ActionExecutor] Connected to action server.")

    def run(self, task_id):
        rospy.loginfo("[ActionExecutor] Preparing goal for task_id=%s" % str(task_id))
        goal = TaskRequestGoal()
        goal.task_id = int(task_id)

        rospy.loginfo("[ActionExecutor] Sending goal")
        self.client.send_goal(goal)
        rospy.loginfo("[ActionExecutor] Waiting for result (timeout based on param)")
        timeout = rospy.get_param("/mas_exec_manager/timeout_sec", 10.0)
        finished = self.client.wait_for_result(rospy.Duration(timeout))

        if not finished:
            rospy.logerr("[ActionExecutor] Action timed out, cancelling")
            self.client.cancel_all_goals()
            return False

        state = self.client.get_state()
        rospy.loginfo("[ActionExecutor] actionlib state=%d" % state)
        return state == GoalStatus.SUCCEEDED
