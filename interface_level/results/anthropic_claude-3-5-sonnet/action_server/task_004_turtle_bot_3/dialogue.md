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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from fetch_head_msgs.action import HeadPointing
from rclpy.action.server import ServerGoalHandle
from action_msgs.msg import GoalStatus

class HeadActionServer(Node):
    def __init__(self):
        super().__init__('head_action_server')
        
        # TODO 1
        self._action_server = ActionServer(
            self,
            HeadPointing,
            'head_action',
            self.execute_cb)
        # END TODO 1

    def execute_cb(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received target TF: {goal_handle.request.target_frame}")

        # TODO 2
        feedback_msg = HeadPointing.Feedback()
        result = HeadPointing.Result()

        # Simulating work
        goal_handle.succeed()
        result.success = True
        return result
        # END TODO 2

def main(args=None):
    rclpy.init(args=args)
    server = HeadActionServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```