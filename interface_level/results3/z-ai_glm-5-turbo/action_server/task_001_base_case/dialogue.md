# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: calibration_action_server_stub.py
----------------------------
#!/usr/bin/env python3
import rospy
import actionlib

from robot_calibration_action.msg import RobotCalibrationAction, RobotCalibrationFeedback, RobotCalibrationResult


class CalibrationActionServer:
    def __init__(self):
        # TODO 1: fill action server name and action type
        self.server = actionlib.SimpleActionServer(
        )

        # start the server (method call) and log startup message

        # log startup message
        # END OF TODO
    def execute_cb(self, goal):
        # TODO 2:
        # instantiate feedback and result with correct classes

        # log received goal contents (goal fields)

        # choose proper loop iterator (e.g., range(0, N))
        # and prepare preemption handling   
        # update feedback fields and numeric progress
        for i in ________:

                return


            # publish feedback
            self.server.publish_feedback(feedback)

            rospy.sleep(10)

        # TODO: fill result fields (success flag and message)

        # log completion

        # mark action succeeded with result
        # END OF TODO

if __name__ == "__main__":
    rospy.init_node("__________")
    CalibrationActionServer()
    rospy.spin()

----------------------------


# LLM Response

[FILENAME: calibration_action_server_stub.py]
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_calibration_action.action import RobotCalibrationAction
from robot_calibration_action.msg import RobotCalibrationFeedback, RobotCalibrationResult


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__("calibration_action_server")
        self.callback_group = ReentrantCallbackGroup()
        # TODO 1: fill action server name and action type
        self.server = ActionServer(
            self,
            RobotCalibrationAction,
            'robot_calibration',
            self.execute_cb,
            callback_group=self.callback_group
        )

        # start the server (method call) and log startup message
        self.get_logger().info("Calibration Action Server started")

        # log startup message
        # END OF TODO
    def execute_cb(self, goal):
        # TODO 2:
        # instantiate feedback and result with correct classes
        feedback = RobotCalibrationFeedback()
        result = RobotCalibrationResult()

        # log received goal contents (goal fields)
        self.get_logger().info(f"Received goal: {goal.request.target}")

        # choose proper loop iterator (e.g., range(0, N))
        # and prepare preemption handling   
        # update feedback fields and numeric progress
        for i in range(0, 10):
            if not self.server.is_active():
                return

            if goal.is_cancel_requested:
                self.get_logger().info('Goal was canceled')
                result.success = False
                result.message = "Calibration canceled"
                goal.canceled()
                return

            # publish feedback
            feedback.progress = i / 10.0
            self.server.publish_feedback(feedback)

            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=10))

        # TODO: fill result fields (success flag and message)
        result.success = True
        result.message = "Calibration completed successfully"

        # log completion
        self.get_logger().info("Calibration completed")

        # mark action succeeded with result
        goal.succeed()
        return result
        # END OF TODO

if __name__ == "__main__":
    rclpy.init()
    node = CalibrationActionServer()
    rclpy.spin(node)
    rclpy.shutdown()