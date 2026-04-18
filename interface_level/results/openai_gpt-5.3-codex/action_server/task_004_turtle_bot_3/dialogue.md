# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python
import rospy
import actionlib
from fetch_head_msgs.msg import HeadPointingAction, HeadPointingResult, HeadPointingFeedback

class HeadActionServer:
    def __init__(self):
        # TODO 1: create a SimpleActionServer for 'head_action'
        # - action type: HeadPointingAction
        # - execute callback
        # - auto_start should be enabled
        # END OF TODO 1
        self.server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received target TF: %s", goal.target_frame)
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        # END of TODO 2


if __name__ == '__main__':
    rospy.init_node('head_action_server')
    server = HeadActionServer()
    rospy.spin()

----------------------------


# LLM Response

#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from fetch_head_msgs.action import HeadPointing


class HeadActionServer(Node):
    def __init__(self):
        super().__init__('head_action_server')
        self.server = ActionServer(
            self,
            HeadPointing,
            'head_action',
            self.execute_cb
        )

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f"Received target TF: {goal.target_frame}")

        result = HeadPointing.Result()
        feedback = HeadPointing.Feedback()

        try:
            for _ in range(5):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)

            goal_handle.succeed()
            return result
        except Exception:
            goal_handle.abort()
            return result


if __name__ == '__main__':
    rclpy.init()
    server = HeadActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()