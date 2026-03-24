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
import actionlib
from robot_calibration_action.action import RobotCalibration

class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__('calibration_action_server')
        self.server = actionlib.SimpleActionServer(
            self, RobotCalibration, 'calibration_action', self.execute_cb, auto_start=False)
        self.server.start()
        self.get_logger().info('Calibration Action Server started.')

    def execute_cb(self, goal):
        feedback = RobotCalibration.Feedback()
        result = RobotCalibration.Result()

        self.get_logger().info(f'Received goal: {goal}')

        for i in range(0, 100):
            if self.server.is_preempt_requested():
                self.get_logger().info('Preempted')
                self.server.set_preempted()
                return

            feedback.progress = i
            self.server.publish_feedback(feedback)
            self.get_logger().info(f'Feedback: {feedback.progress}%')
            rclpy.sleep(1)

        result.success = True
        result.message = 'Calibration completed successfully.'
        self.get_logger().info('Calibration completed.')
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rclpy.init()
    server = CalibrationActionServer()
    rclpy.spin(server)
    rclpy.shutdown()
```