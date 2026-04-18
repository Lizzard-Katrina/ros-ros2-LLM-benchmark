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