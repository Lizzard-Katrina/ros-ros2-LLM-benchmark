# Task: action_server/task_004_turtle_bot_3

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
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
            execute_callback=self.execute_cb)
        # END OF TODO 1

    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f"Received target TF: {goal.target_frame}")
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        feedback_msg = HeadPointing.Feedback()
        result = HeadPointing.Result()

        # Simulate some feedback and processing
        for i in range(1, 6):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return result
            feedback_msg.progress = i * 20  # example progress percentage
            goal_handle.publish_feedback(feedback_msg)
            await rclpy.sleep(0.2)

        result.success = True
        goal_handle.succeed()
        return result
        # END of TODO 2


def main(args=None):
    rclpy.init(args=args)
    server = HeadActionServer()
    rclpy.spin(server)
    server.server.destroy()
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```