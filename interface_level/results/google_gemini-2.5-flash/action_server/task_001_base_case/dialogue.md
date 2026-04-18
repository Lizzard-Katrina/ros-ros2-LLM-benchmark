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
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from robot_calibration_action.action import RobotCalibration


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__('calibration_action_server')
        # TODO 1: fill action server name and action type
        self.server = ActionServer(
            self,
            RobotCalibration,
            'robot_calibration',
            self.execute_cb
        )

        self.get_logger().info("Robot Calibration Action Server Started")
        # END OF TODO

    def execute_cb(self, goal_handle: ServerGoalHandle):
        # TODO 2:
        # instantiate feedback and result with correct classes
        feedback_msg = RobotCalibration.Feedback()
        result_msg = RobotCalibration.Result()

        # log received goal contents (goal fields)
        self.get_logger().info(f"Received goal: {goal_handle.request.command}")

        # choose proper loop iterator (e.g., range(0, N))
        # and prepare preemption handling
        # update feedback fields and numeric progress
        for i in range(0, 10): # Example: 10 steps
            if not rclpy.ok():
                self.get_logger().info('Action server shutting down, pre-empting goal.')
                return RobotCalibration.Result() # Return an empty result or a specific preemption result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return RobotCalibration.Result()

            feedback_msg.progress = float(i / 10.0)
            feedback_msg.status_message = f"Calibrating step {i+1} of 10"
            # publish feedback
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f"Publishing feedback: {feedback_msg.progress}")
            rclpy.sleep(1.0) # Simulate work

        # TODO: fill result fields (success flag and message)
        result_msg.success = True
        result_msg.message = "Calibration completed successfully!"

        # log completion
        self.get_logger().info(f"Goal completed: {result_msg.message}")

        # mark action succeeded with result
        goal_handle.succeed()
        return result_msg
        # END OF TODO

if __name__ == "__main__":
    rclpy.init()
    node = CalibrationActionServer()
    rclpy.spin(node)
    rclpy.shutdown()
```