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
            self.execute_cb
        )
        # END OF TODO 1
        # self.server.start()  # Note: auto_start is default in ROS2, start() is not needed

    def execute_cb(self, goal_handle):
        self.get_logger().info("Received target TF: %s" % goal_handle.request.target_frame)
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        feedback_msg = HeadPointing.Feedback()
        
        # Simulate publishing feedback
        goal_handle.publish_feedback(feedback_msg)
        
        # Mark the goal as successful
        goal_handle.succeed()
        
        # Populate and return the result
        result = HeadPointing.Result()
        return result
        # END of TODO 2


def main(args=None):
    rclpy.init(args=args)
    server = HeadActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```