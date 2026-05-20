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
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from fetch_head_msgs.action import HeadPointing
from fetch_head_msgs.msg import HeadPointingFeedback, HeadPointingResult

class HeadActionServer:
    def __init__(self):
        self.node = rclpy.create_node('head_action_server')
        
        # TODO 1: create a SimpleActionServer for 'head_action'
        # - action type: HeadPointingAction
        # - execute callback
        # - auto_start should be enabled
        self.server = ActionServer(
            self.node,
            HeadPointing,
            'head_action',
            self.execute_cb,
            callback_group=ReentrantCallbackGroup()
        )
        # END OF TODO 1
        # In ROS2 ActionServer, start() is called implicitly upon creation if not deferred,
        # but we explicitly ensure it's ready or rely on the constructor behavior.
        # The original ROS1 code called start(), so we ensure the server is active.
        # ActionServer in ROS2 starts listening immediately upon creation.

    def execute_cb(self, goal_handle):
        self.node.get_logger().info("Received target TF: %s" % goal_handle.request.target_frame)
        
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        feedback = HeadPointingFeedback()
        result = HeadPointingResult()
        
        # Simulate processing steps
        for i in range(1, 4):
            if goal_handle.is_cancel_requested:
                self.node.get_logger().info('Goal canceled')
                goal_handle.canceled()
                return result
            
            feedback.percent_complete = i * 33
            self.node.get_logger().info('Publishing feedback: %d%%' % feedback.percent_complete)
            goal_handle.publish_feedback(feedback)
            rclpy.sleep_for(seconds=1)

        result.success = True
        goal_handle.succeed(result)
        return result
        # END of TODO 2


if __name__ == '__main__':
    rclpy.init()
    server = HeadActionServer()
    try:
        rclpy.spin(server.node)
    except KeyboardInterrupt:
        pass
    finally:
        server.server.destroy()
        server.node.destroy_node()
        rclpy.shutdown()