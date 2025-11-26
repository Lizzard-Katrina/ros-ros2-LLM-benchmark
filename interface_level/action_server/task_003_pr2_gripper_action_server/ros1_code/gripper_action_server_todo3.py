#!/usr/bin/env python
import rospy
import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandResult, Pr2GripperCommandFeedback

class GripperActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'gripper_action',
            Pr2GripperCommandAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received goal: position=%.2f effort=%.2f", goal.command.position, goal.command.max_effort)
        for i in range(5):
            feedback = Pr2GripperCommandFeedback()
            feedback.position = goal.command.position * (i + 1) / 5.0
            # TODO: publish feedback to client
            # end:   # task ends here
            rospy.sleep(0.2)
        result = Pr2GripperCommandResult()
        result.position = goal.command.position
        result.effort = goal.command.max_effort
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('pr2_gripper_action_server')
    server = GripperActionServer()
    rospy.spin()
