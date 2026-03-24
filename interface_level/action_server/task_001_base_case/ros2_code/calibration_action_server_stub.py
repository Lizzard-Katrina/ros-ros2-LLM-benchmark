Here is the converted ROS2 code:
```python
#!/usr/bin/env python3
import rclpy
import rclpy.action
from rclpy.action import ActionServer
from rclpy.node import Node
from robot_calibration_action.action import RobotCalibration
from robot_calibration_action.msg import RobotCalibrationFeedback, RobotCalibrationResult


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__('calibration_action_server')
        # TODO 1: fill action server name and action type
        self.server = ActionServer(
            self,
            RobotCalibration,
            'calibration_action',
            self.execute_cb
        )

        # log startup message
        self.get_logger().info('Calibration Action Server started')

    def execute_cb(self, goal_handle):
        # TODO 2:
        # instantiate feedback and result with correct classes
        feedback = RobotCalibrationFeedback()
        result = RobotCalibrationResult()

        # log received goal contents (goal fields)
        self.get_logger().info('Received goal: %s' % goal_handle.request)

        # choose proper loop iterator (e.g., range(0, N))
        # and prepare preemption handling   
        for i in range(10):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                return

            # update feedback fields and numeric progress
            feedback.progress = i
            feedback.message = 'Processing...'

            # publish feedback
            goal_handle.publish_feedback(feedback)

            self.get_logger().info('Publishing feedback: %s' % feedback)

            rclpy.spin_once(self, timeout_sec=10)

        # TODO: fill result fields (success flag and message)
        result.success = True
        result.message = 'Calibration completed'

        # log completion
        self.get_logger().info('Calibration completed')

        # mark action succeeded with result
        goal_handle.succeed()
        return result

if __name__ == "__main__":
    rclpy.init()
    calibration_action_server = CalibrationActionServer()
    rclpy.spin(calibration_action_server)
```
Note that I've replaced `rospy` with `rclpy`, and `actionlib` with `rclpy.action`. I've also updated the node name, action server name, and action type to match the ROS2 conventions. Additionally, I've replaced `rospy.sleep` with `rclpy.spin_once` to achieve similar functionality.