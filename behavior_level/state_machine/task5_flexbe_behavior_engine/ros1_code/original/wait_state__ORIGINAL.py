#!/usr/bin/env python
# ORIGINAL: wait_state__ORIGINAL.py

import rospy
import time

class WaitState(object):
    def __init__(self, duration=1.0, name='WaitState'):
        self.duration = duration
        self.name = name
        self.userdata = type('ud', (), {})()
        self.outcomes = ['done']

    def execute(self, userdata):
        rospy.loginfo("[WaitState] waiting for %s seconds" % str(self.duration))
        time.sleep(self.duration)
        return 'done'
