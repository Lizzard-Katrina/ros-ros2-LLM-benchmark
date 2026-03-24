# Task: action_server/task_001_base_case

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from robot_calibration_action.action import RobotCalibration


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__('calibration_action_server')
        self._action_server = ActionServer(
            self,
            RobotCalibration,
            'calibration_action',
            execute_callback=self.execute_cb)

        self.get_logger().info('Calibration Action Server started')

    async def execute_cb(self, goal_handle):
        feedback_msg = RobotCalibration.Feedback()
        result = RobotCalibration.Result()

        self.get_logger().info(f'Received goal: {goal_handle.request}')

        for i in range(0, 100):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return RobotCalibration.Result()

            feedback_msg.progress = i
            feedback_msg.status = f'Progress: {i}%'
            goal_handle.publish_feedback(feedback_msg)

            await rclpy.sleep(0.1)

        result.success = True
        result.message = 'Calibration completed successfully'

        self.get_logger().info('Calibration succeeded')

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    calibration_action_server = CalibrationActionServer()
    rclpy.spin(calibration_action_server)
    calibration_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```