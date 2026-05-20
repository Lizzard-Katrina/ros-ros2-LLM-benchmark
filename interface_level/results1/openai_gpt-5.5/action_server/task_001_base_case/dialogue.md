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
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node

from robot_calibration_action.action import RobotCalibration


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__("calibration_action_server")

        self.server = ActionServer(
            self,
            RobotCalibration,
            "robot_calibration",
            self.execute_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info("Calibration action server started")

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Calibration goal cancel request received")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        feedback = RobotCalibration.Feedback()
        result = RobotCalibration.Result()

        goal = goal_handle.request
        self.get_logger().info(f"Received calibration goal: {goal}")

        for i in range(0, 10):
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = "Calibration canceled"
                goal_handle.canceled()
                self.get_logger().info("Calibration canceled")
                return result

            if hasattr(feedback, "progress"):
                feedback.progress = float(i + 1) / 10.0
            if hasattr(feedback, "message"):
                feedback.message = f"Calibration in progress: {i + 1}/10"

            goal_handle.publish_feedback(feedback)

            rclpy.sleep(10.0)

        result.success = True
        result.message = "Calibration completed successfully"

        self.get_logger().info("Calibration completed successfully")

        goal_handle.succeed()
        return result


if __name__ == "__main__":
    rclpy.init()
    node = CalibrationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
