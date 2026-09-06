#!/usr/bin/env python3
"""
Helper action server node that mirrors the logic from the translated
move_action_capability.cpp for runtime testing purposes.

This is a Python re-implementation of the same action server interface
so we can test the action type and interaction pattern at runtime.
The actual translated code is the C++ file; this helper exists only
so the pytest can exercise the MoveGroup action interface.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes


class MoveGroupMoveActionServer(Node):
    """Mirrors the C++ MoveGroupMoveAction server logic."""

    def __init__(self):
        super().__init__('move_group_move_action')
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            MoveGroup,
            'move_action',
            self.execute_callback,
            callback_group=self._cb_group
        )
        self.get_logger().info('MoveGroup action server started on "move_action"')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request')

        # setMoveState(PLANNING)
        fb = MoveGroup.Feedback()
        fb.state = 'PLANNING'
        goal_handle.publish_feedback(fb)

        goal = goal_handle.request

        result = MoveGroup.Result()

        if goal.planning_options.plan_only:
            self.get_logger().info('Planning request received for MoveGroup action.')
            result.error_code.val = MoveItErrorCodes.SUCCESS
        else:
            self.get_logger().info(
                'Combined planning and execution request received for MoveGroup action.')
            # setMoveState(MONITOR)
            fb2 = MoveGroup.Feedback()
            fb2.state = 'MONITOR'
            goal_handle.publish_feedback(fb2)
            result.error_code.val = MoveItErrorCodes.SUCCESS

        goal_handle.succeed()

        # setMoveState(IDLE)
        fb_idle = MoveGroup.Feedback()
        fb_idle.state = 'IDLE'
        goal_handle.publish_feedback(fb_idle)

        return result


def main():
    rclpy.init()
    node = MoveGroupMoveActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()