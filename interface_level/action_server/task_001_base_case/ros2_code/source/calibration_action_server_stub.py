#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import rclpy.action
import time

from task_001_base_case.action import RobotCalibration


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__('calibration_action_server')
        self._action_server = ActionServer(
            self,
            RobotCalibration,
            'robot_calibration',
            self.execute_cb
        )
        self.get_logger().info('Calibration action server started')

    def execute_cb(self, goal_handle):
        feedback = RobotCalibration.Feedback()
        result = RobotCalibration.Result()

        num_steps = goal_handle.request.num_steps
        calibration_type = goal_handle.request.calibration_type
        self.get_logger().info(
            f'Received goal: num_steps={num_steps}, calibration_type={calibration_type}'
        )

        i = 0
        while (i < num_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Calibration canceled'
                self.get_logger().info('Goal canceled')
                return result

            percent = float(i + 1) / float(num_steps) * 100.0
            feedback.percent_complete = percent
            feedback.current_step = f'Step {i + 1} of {num_steps}'
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Feedback: {feedback.percent_complete}%')

            time.sleep(0.1)
            i += 1

        result.success = True
        result.message = 'Calibration completed successfully'
        self.get_logger().info('Calibration completed')
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()