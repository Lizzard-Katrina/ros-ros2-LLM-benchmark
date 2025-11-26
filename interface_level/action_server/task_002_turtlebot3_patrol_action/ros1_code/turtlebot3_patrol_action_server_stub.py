#!/usr/bin/env python3
import rospy
import actionlib
# TODO: import correct action type
from turtlebot3_msgs.msg import PatrolAction, PatrolFeedback, PatrolResult

class PatrolActionServer:
    def __init__(self):
        # TODO: action server name
        self.server = actionlib.SimpleActionServer(
            "_________",
            PatrolAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("TurtleBot3 Patrol Action Server started.")

    def execute_cb(self, goal):
        feedback = PatrolFeedback()
        result = PatrolResult()

        # TODO: fill patrol sequence loop
        for i, waypoint in enumerate(goal.waypoints):
            # TODO: update feedback for each waypoint
            feedback._________ = f"Moving to waypoint {i}"
            self.server.publish_feedback(feedback)
            rospy.sleep(1.0)  # simulate patrol step

        # TODO: fill result
        result._________ = True
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("turtlebot3_patrol_action_server")
    PatrolActionServer()
    rospy.spin()
