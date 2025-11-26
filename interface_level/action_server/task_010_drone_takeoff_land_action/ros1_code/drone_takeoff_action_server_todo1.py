#!/usr/bin/env python
import rospy
import actionlib
from px4_ros_com.msg import TakeoffLandAction, TakeoffLandResult, TakeoffLandFeedback

class DroneTakeoffActionServer:
    def __init__(self):
        # TODO: create SimpleActionServer instance
        # end:   # task ends here
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received takeoff/land goal")
        for i in range(5):
            feedback = TakeoffLandFeedback()
            feedback.altitude = 0.2 * i  # dummy feedback
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = TakeoffLandResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('drone_takeoff_action_server')
    server = DroneTakeoffActionServer()
    rospy.spin()
