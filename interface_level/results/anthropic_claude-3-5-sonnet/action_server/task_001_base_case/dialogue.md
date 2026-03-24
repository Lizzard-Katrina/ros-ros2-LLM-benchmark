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
from rclpy.callback_groups import ReentrantCallbackGroup


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__('calibration_action_server')
        self._action_server = ActionServer(
            self,
            RobotCalibration,
            'robot_calibration',
            self.execute_cb,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info('Calibration action server started')

    async def execute_cb(self, goal_handle: ServerGoalHandle):
        feedback = RobotCalibration.Feedback()
        result = RobotCalibration.Result()

        self.get_logger().info(f'Executing goal: {goal_handle.request}')

        for i in range(0, 100):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return RobotCalibration.Result()

            feedback.progress = i
            feedback.status = "Calibrating..."
            goal_handle.publish_feedback(feedback)

            self.create_rate(0.1).sleep()

        result.success = True
        result.message = "Calibration completed successfully"

        self.get_logger().info('Goal succeeded')

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = CalibrationActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```