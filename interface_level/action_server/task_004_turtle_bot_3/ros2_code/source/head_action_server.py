#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from task_004_turtle_bot_3.action import HeadPointing


class HeadActionServer(Node):
    def __init__(self):
        super().__init__('head_action_server')
        self.server = ActionServer(
            self,
            HeadPointing,
            'head_action',
            self.execute_callback
        )
        self.get_logger().info('Head action server is ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info(
            'Received target TF: %s' % goal_handle.request.target_frame
        )

        feedback_msg = HeadPointing.Feedback()
        total_steps = 10

        for i in range(total_steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = HeadPointing.Result()
                result.success = False
                result.message = 'Goal was canceled'
                return result

            feedback_msg.progress = float(i + 1) / float(total_steps)
            feedback_msg.status = 'Pointing to %s: step %d/%d' % (
                goal_handle.request.target_frame, i + 1, total_steps
            )
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                'Feedback: progress=%.1f' % feedback_msg.progress
            )
            time.sleep(0.1)

        result = HeadPointing.Result()
        result.success = True
        result.message = 'Successfully pointed to %s' % goal_handle.request.target_frame
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HeadActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()