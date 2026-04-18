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
        # TODO 1: fill action server name and action type
        super().__init__('calibration_action_server')
        self.server = ActionServer(
            self,
            RobotCalibrationAction,
            'calibration_action',
            self.execute_cb
        )

        # log startup message
        self.get_logger().info('Calibration Action Server started')

    def execute_cb(self, goal):
        # TODO 2:
        # instantiate feedback and result with correct classes
        feedback = RobotCalibrationFeedback()
        result = RobotCalibrationResult()

        # log received goal contents (goal fields)
        self.get_logger().info(f'Received goal: {goal}')

        # choose proper loop iterator (e.g., range(0, N))
        # and prepare preemption handling   
        for i in range(0, 10):
            # update feedback fields and numeric progress
            feedback.progress = i
            if self.server.is_preempt_requested():
                self.server.cancel_goal()
                self.get_logger().info('Goal cancelled')
                return RobotCalibrationResult()

            # publish feedback
            self.server.publish_feedback(feedback)

            self.get_logger().info(f'Publishing feedback: {feedback}')
            rclpy.spin_once(self, timeout_sec=0.1)

        # TODO: fill result fields (success flag and message)
        result.success = True
        result.message = 'Calibration completed successfully'

        # log completion
        self.get_logger().info('Calibration completed')

        # mark action succeeded with result
        return result

if __name__ == "__main__":
    rclpy.init()
    node = CalibrationActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```