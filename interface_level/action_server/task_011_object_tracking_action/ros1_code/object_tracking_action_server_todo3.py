#!/usr/bin/env python
import rospy
import actionlib
from moveit_tutorials.msg import ObjectTrackingAction, ObjectTrackingResult, ObjectTrackingFeedback

class ObjectTrackingActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'object_tracking_action',
            ObjectTrackingAction,
            execute_cb=self.execute_cb,
            auto_start=True
        )

    def execute_cb(self, goal):
        # TODO: parse goal (object id, detection parameters)
        # end:   # task ends here
        rospy.loginfo("Parsed object tracking goal")
        for i in range(5):
            feedback = ObjectTrackingFeedback()
            feedback.pose = [0.0, 0.0, 0.0]
            self.server.publish_feedback(feedback)
            rospy.sleep(0.2)
        result = ObjectTrackingResult()
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('object_tracking_action_server')
    server = ObjectTrackingActionServer()
    rospy.spin()
