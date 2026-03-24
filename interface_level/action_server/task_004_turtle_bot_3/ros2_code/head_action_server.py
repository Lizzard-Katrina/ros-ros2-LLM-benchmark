#!/usr/bin/env python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from fetch_head_msgs.action import HeadPointing
from rclpy.executors import ExternalShutdownException

class HeadActionServer(Node):
    def __init__(self):
        super().__init__('head_action_server')
        # TODO 1: create a SimpleActionServer for 'head_action'
        # - action type: HeadPointing
        # - execute callback
        # - auto_start should be enabled
        self.server = ActionServer(
            self,
            HeadPointing,
            'head_action',
            self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )
        # END OF TODO 1

    def goal_cb(self, goal_request):
        self.get_logger().info('Received goal request')
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return rclpy.action.server.CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        self.get_logger().info("Received target TF: %s", goal_handle.request.target_frame)
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        # END of TODO 2
        goal_handle.succeed()
        result = HeadPointing.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    server = HeadActionServer()
    try:
        rclpy.spin(server)
    except ExternalShutdownException:
        pass
    finally:
        server.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()