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
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from robot_calibration_action.action import RobotCalibration


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__("calibration_action_server")
        self._current_goal_handle = None

        self.server = ActionServer(
            self,
            RobotCalibration,
            "robot_calibration",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("Calibration action server started")

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received goal request: target_position={getattr(goal_request, 'target_position', None)}, "
            f"calibration_type={getattr(goal_request, 'calibration_type', None)}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        self._current_goal_handle = goal_handle

        feedback = RobotCalibration.Feedback()
        result = RobotCalibration.Result()

        self.get_logger().info(
            f"Executing goal: target_position={getattr(goal_handle.request, 'target_position', None)}, "
            f"calibration_type={getattr(goal_handle.request, 'calibration_type', None)}"
        )

        for i in range(0, 10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Calibration canceled"
                return result

            if hasattr(feedback, "current_step"):
                feedback.current_step = i + 1
            if hasattr(feedback, "progress"):
                feedback.progress = float((i + 1) * 10.0)

            goal_handle.publish_feedback(feedback)

            rclpy.spin_once(self, timeout_sec=0.0)
            import time
            time.sleep(10)

        result.success = True
        result.message = "Calibration completed successfully"

        self.get_logger().info("Calibration completed")

        goal_handle.succeed()
        return result


if __name__ == "__main__":
    rclpy.init()
    node = CalibrationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()