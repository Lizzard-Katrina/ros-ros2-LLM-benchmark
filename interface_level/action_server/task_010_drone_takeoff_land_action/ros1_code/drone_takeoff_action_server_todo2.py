#!/usr/bin/env python
import rospy
import actionlib
from px4_ros_com.msg import TakeoffLandAction, TakeoffLandResult, TakeoffLandFeedback

class DroneTakeoffActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'drone_takeoff_action',
            TakeoffLandAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        # TODO: start the server
        # end:   # task ends here

    def execute_cb(self, goal):
        rospy.loginfo("Received takeoff/land goal")
        for i in range(5):
            feedback = TakeoffLandFeedback()
            feedback.altitude = 0.2 * i
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = TakeoffLandResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('drone_takeoff_action_server')
    server = DroneTakeoffActionServer()
    rospy.spin()

