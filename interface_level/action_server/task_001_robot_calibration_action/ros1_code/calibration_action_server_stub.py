#!/usr/bin/env python3
import rospy
import actionlib

# TODO: import correct action classes (Action, Feedback, Result)
# from <your_pkg>.msg import RobotCalibrationAction, RobotCalibrationFeedback, RobotCalibrationResult
from robot_calibration_action.msg import ________, ________, ________


class CalibrationActionServer:
    def __init__(self):
        # TODO: fill action server name and action type
        self.server = actionlib.SimpleActionServer(
            "__________",   # action server name
            ________,       # Action type
            execute_cb=self.execute_cb,
            auto_start=False
        )

        # TODO: start the server (method call)
        ________

        # TODO: log startup message
        rospy.loginfo("__________")

    def execute_cb(self, goal):
        # TODO: instantiate feedback/result with correct classes
        feedback = ________()
        result = ________()

        # TODO: log received goal contents (goal fields)
        rospy.loginfo("Calibration started: target_accuracy=%s, mode=%s", ________, ________)

        # TODO: choose proper loop iterator (e.g., range(0, N))
        for i in ________:

            # TODO: preemption handling (server preempt request)
            if ________:
                rospy.logwarn("__________")
                # TODO: set preempted appropriately
                ________
                return

            # TODO: update feedback fields (name + content)
            feedback.__________ = "Step %d: reading sensors..." % ________
            # TODO: also update numeric progress if available
            feedback.__________ = float(i) / (________)  # e.g., progress in [0,1]

            # publish feedback
            self.server.publish_feedback(feedback)

            # TODO: sleep for an appropriate amount
            rospy.sleep(__________)

        # TODO: fill result fields (success flag and message)
        result.__________ = ________   # success True/False
        result.__________ = "__________"  # human-readable message

        # TODO: log completion
        rospy.loginfo("__________")

        # TODO: mark action succeeded with result
        ________(result)


if __name__ == "__main__":
    rospy.init_node("__________")
    CalibrationActionServer()
    rospy.spin()
