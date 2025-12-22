
import rospy
import time

class WaitState(object):
    def __init__(self, duration=0.5, name='WaitState'):
        self.duration = duration
        self.name = name

    def on_entry(self):
        rospy.loginfo("[WaitState] on_entry: waiting %s" % str(self.duration))
        time.sleep(self.duration)

class LogState(object):
    def __init__(self, message="", name='LogState'):
        self.message = message
        self.name = name

    def on_entry(self):
        rospy.loginfo("[LogState] %s" % self.message)
