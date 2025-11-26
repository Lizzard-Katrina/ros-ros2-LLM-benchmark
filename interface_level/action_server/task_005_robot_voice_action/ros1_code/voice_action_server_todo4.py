#!/usr/bin/env python
import rospy
import actionlib
from voice_command_msgs.msg import VoiceCommandAction, VoiceCommandResult, VoiceCommandFeedback

class VoiceActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'voice_action',
            VoiceCommandAction,
            execute_cb=self.execute_cb,
            auto_start=True
        )

    def execute_cb(self, goal):
        rospy.loginfo("Received command: %s", goal.command)
        for i in range(5):
            feedback = VoiceCommandFeedback()
            feedback.progress = (i + 1) / 5.0
            # TODO: publish feedback to client
            # end:   # task ends here
            rospy.sleep(0.2)
        result = VoiceCommandResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('voice_action_server')
    server = VoiceActionServer()
    rospy.spin()
