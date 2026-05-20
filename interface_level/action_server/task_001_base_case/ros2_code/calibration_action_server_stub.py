#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import time

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

        # start the server (method call) and log startup message
        self.get_logger().info("Calibration action server started.")
        # END OF TODO

    def execute_cb(self, goal_handle):
        # TODO 2:
        # instantiate feedback and result with correct classes
        feedback = RobotCalibration.Feedback()
        result = RobotCalibration.Result()

        # log received goal contents (goal fields)
        self.get_logger().info(f"Received goal request: {goal_handle.request}")

        # choose proper loop iterator (e.g., range(0, N))
        # and prepare preemption handling   
        # update feedback fields and numeric progress
        for i in range(1, 6):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return result

            feedback.progress = float(i * 20)
            # publish feedback
            goal_handle.publish_feedback(feedback)

            time.sleep(10)

        # TODO: fill result fields (success flag and message)
        result.success = True
        result.message = "Calibration completed successfully"

        # log completion
        self.get_logger().info("Calibration completed")

        # mark action succeeded with result
        goal_handle.succeed()
        return result
        # END OF TODO

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()