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
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from fetch_head_msgs.action import HeadPointing

class HeadActionServer(Node):
    def __init__(self):
        super().__init__('head_action_server')
        # TODO 1: create a SimpleActionServer for 'head_action'
        # - action type: HeadPointingAction
        # - execute callback
        # - auto_start should be enabled
        self.server = ActionServer(
            self,
            HeadPointing,
            'head_action',
            self.execute_cb,
            callback_group=ReentrantCallbackGroup()
        )
        # END OF TODO 1

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f"Received target TF: {goal.target_frame}")
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        goal_handle.execute()
        
        feedback_msg = HeadPointing.Feedback()
        goal_handle.publish_feedback(feedback_msg)
        
        result = HeadPointing.Result()
        goal_handle.succeed()
        return result
        # END of TODO 2


if __name__ == '__main__':
    rclpy.init()
    server = HeadActionServer()
    rclpy.spin(server)