#!/usr/bin/env python
import rospy
import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

def handle_feedback(feedback):
    # TODO: process feedback
    # end:   # task ends here

if __name__ == '__main__':
    rospy.init_node('pr2_gripper_action_client')
    client = actionlib.SimpleActionClient('gripper_action', Pr2GripperCommandAction)
    # TODO: wait for server
    # end:   # task ends here
    goal = Pr2GripperCommandGoal()
    goal.command.position = 0.1
    goal.command.max_effort = 50.0
    client.send_goal(goal, feedback_cb=handle_feedback)
    client.wait_for_result()
