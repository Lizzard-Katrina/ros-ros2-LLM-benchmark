#!/usr/bin/env python
import rospy
import actionlib
from voice_command_msgs.msg import VoiceCommandAction, VoiceCommandResult, VoiceCommandFeedback

class VoiceActionServer:
    def __init__(self):
        # TODO: create SimpleActionServer instance
        # end:   # task ends here
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received command: %s", goal.command)
        # simulate command execution
        for i in range(5):
            feedback = VoiceCommandFeedback()
            feedback.progress = (i + 1) / 5.0
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = VoiceCommandResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('voice_action_server')
    server = VoiceActionServer()
    rospy.spin()
