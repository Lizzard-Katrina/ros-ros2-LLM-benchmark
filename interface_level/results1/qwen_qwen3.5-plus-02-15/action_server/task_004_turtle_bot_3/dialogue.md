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
from rclpy.action import ActionServer
from rclpy.node import Node
from fetch_head_msgs.action import HeadPointing
import time

class HeadActionServer:
    def __init__(self):
        self.node = rclpy.create_node('head_action_server')
        # TODO 1: create a SimpleActionServer for 'head_action'
        # - action type: HeadPointingAction
        # - execute callback
        # - auto_start should be enabled
        self.server = ActionServer(self.node, HeadPointing, 'head_action', self.execute_cb)
        # END OF TODO 1
        # self.server.start()  # ActionServer starts automatically in ROS2

    def execute_cb(self, goal_handle):
        self.node.get_logger().info("Received target TF: %s", goal_handle.goal.target_frame)
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        feedback = HeadPointing.Feedback()
        result = HeadPointing.Result()
        
        # Simulate execution
        for i in range(1, 10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.node.get_logger().info('Goal canceled')
                return result
            
            feedback.current_frame = "frame_%d" % i
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
        
        goal_handle.succeed()
        self.node.get_logger().info('Goal succeeded')
        return result
        # END of TODO 2


if __name__ == '__main__':
    rclpy.init()
    server = HeadActionServer()
    rclpy.spin(server.node)
    server.node.destroy_node()
    rclpy.shutdown()