#!/usr/bin/env python
import rospy
import actionlib
from moveit_tutorials.msg import ObjectTrackingAction, ObjectTrackingResult, ObjectTrackingFeedback

class ObjectTrackingActionServer:
    def __init__(self):
        # TODO: create SimpleActionServer instance
        # end:   # task ends here
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received object tracking goal")
        for i in range(5):
            feedback = ObjectTrackingFeedback()
            feedback.pose = [0.0, 0.0, 0.0]  # dummy pose
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = ObjectTrackingResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('object_tracking_action_server')
    server = ObjectTrackingActionServer()
    rospy.spin()
