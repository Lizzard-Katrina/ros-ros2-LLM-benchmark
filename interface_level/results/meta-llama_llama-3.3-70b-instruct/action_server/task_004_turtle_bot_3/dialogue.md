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

```python
#!/usr/bin/env python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from fetch_head_msgs.action import HeadPointing
from fetch_head_msgs.msg import HeadPointingFeedback, HeadPointingResult

class HeadActionServer(Node):
    def __init__(self):
        super().__init__('head_action_server')
        # TODO 1: create a SimpleActionServer for 'head_action'
        # - action type: HeadPointing
        # - execute callback
        # - auto_start should be enabled
        self.server = ActionServer(
            self,
            HeadPointing,
            'head_action',
            self.execute_cb,
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        # END OF TODO 1

    def execute_cb(self, goal_handle):
        self.get_logger().info(f"Received target TF: {goal_handle.request.target_frame}")
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        feedback_msg = HeadPointingFeedback()
        result = HeadPointingResult()
        while rclpy.ok():
            # simulate work
            self.get_logger().info(f"Head pointing to {goal_handle.request.target_frame}")
            feedback_msg.percent_complete = 50
            goal_handle.publish_feedback(feedback_msg)
            # check if goal was cancelled
            if goal_handle.is_cancel_requested():
                self.get_logger().info("Goal was cancelled")
                goal_handle.canceled()
                return HeadPointing.Result()
            # check if goal was preempted
            if goal_handle.is_preempt_requested():
                self.get_logger().info("Goal was preempted")
                goal_handle.preempt()
                return HeadPointing.Result()
            # simulate completion
            result.success = True
            goal_handle.succeed()
            return result
        # END of TODO 2


if __name__ == '__main__':
    rclpy.init()
    server = HeadActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        server.destroy_node()
        rclpy.shutdown()
```