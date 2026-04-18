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

#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

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
        self.get_logger().info('Calibration Action Server started')
        # END OF TODO

    def execute_cb(self, goal_handle):
        # TODO 2:
        # instantiate feedback and result with correct classes
        feedback_msg = RobotCalibration.Feedback()
        result = RobotCalibration.Result()

        # log received goal contents (goal fields)
        self.get_logger().info(f'Received goal: {goal_handle.request}')

        # choose proper loop iterator (e.g., range(0, N))
        # and prepare preemption handling
        for i in range(0, 10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return

            # update feedback fields and numeric progress
            feedback_msg.progress = i * 10
            feedback_msg.current_step = f'Step {i}'

            # publish feedback
            goal_handle.publish_feedback(feedback_msg)

            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=10))

        # TODO: fill result fields (success flag and message)
        result.success = True
        result.message = 'Calibration completed successfully'

        # log completion
        self.get_logger().info('Calibration completed')

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