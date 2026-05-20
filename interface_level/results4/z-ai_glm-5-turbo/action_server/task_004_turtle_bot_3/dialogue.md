# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: head_action_server_todo.py
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

[FILENAME: head_action_server_todo.py]
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from fetch_head_msgs.action import HeadPointing
from fetch_head_msgs.msg import HeadPointingResult, HeadPointingFeedback

class HeadActionServer(Node):
    def __init__(self):
        super().__init__('head_action_server')
        # TODO 1: create a SimpleActionServer for 'head_action'
        # - action type: HeadPointingAction
        # - execute callback
        # - auto_start should be enabled
        # END OF TODO 1
        self._action_server = ActionServer(
            self,
            HeadPointing,
            'head_action',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            auto_start=True
        )

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        self.get_logger().info("Received target TF: %s", goal_handle.request.target_frame)
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        # END of TODO 2
        result = HeadPointingResult()
        feedback = HeadPointingFeedback()
        
        result.success = True
        
        if not goal_handle.is_active:
            self.get_logger().info("Goal was aborted before execution started")
            return result
            
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info("Goal canceled")
            result.success = False
            return result

        for i in range(1, 11):
            if not goal_handle.is_active:
                break
                
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                result.success = False
                return result
                
            feedback.pointing_angle = float(i) / 10.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("Publishing feedback: %f" % feedback.pointing_angle)
            
            import time
            time.sleep(0.1)

        if goal_handle.is_active:
            goal_handle.succeed()
            self.get_logger().info("Goal succeeded")
        
        return result


if __name__ == '__main__':
    rclpy.init()
    server = HeadActionServer()
    rclpy.spin(server)
    rclpy.shutdown()