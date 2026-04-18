# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
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

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from robot_calibration_action.msg import RobotCalibrationAction, RobotCalibrationFeedback, RobotCalibrationResult


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__("calibration_action_server")

        # TODO 1: fill action server name and action type
        self.server = ActionServer(
            node=self,
            action_type=RobotCalibrationAction,
            action_name='calibrate',
            execute_callback=self.execute_cb
        )

        # start the server (method call) and log startup message

        # log startup message
        self.get_logger().info("Calibration Action Server started")
        # END OF TODO

    def execute_cb(self, goal_handle):
        # TODO 2:
        # instantiate feedback and result with correct classes
        feedback = RobotCalibrationFeedback()
        result = RobotCalibrationResult()

        # log received goal contents (goal fields)
        self.get_logger().info(f"Received goal: {goal_handle.request}")

        # choose proper loop iterator (e.g., range(0, N))
        # and prepare preemption handling
        # update feedback fields and numeric progress
        for i in range(0, 10):

            # Check for preemption
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return RobotCalibrationResult()

            # publish feedback
            feedback.progress = i
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f"Feedback published: {feedback.progress}")

            rclpy.sleep(1.0)

        # TODO: fill result fields (success flag and message)
        result.success = True
        result.message = "Calibration completed successfully"

        # log completion
        self.get_logger().info("Calibration completed")

        # mark action succeeded with result
        goal_handle.succeed(result)
        return result
        # END OF TODO


if __name__ == "__main__":
    rclpy.init()
    server = CalibrationActionServer()
    rclpy.spin(server)
    rclpy.shutdown()
```