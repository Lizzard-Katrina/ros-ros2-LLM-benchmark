#!/usr/bin/env python3
import time
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

    def execute_cb(self, goal_handle):
        self.get_logger().info("Received target TF: %s" % goal_handle.request.target_frame)
        # TODO 2: handle the full lifecycle of a head-pointing action goal
        feedback_msg = HeadPointing.Feedback()
        
        for i in range(1, 4):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return HeadPointing.Result()
            
            # Simulate some work and publish feedback
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
            
        goal_handle.succeed()
        result = HeadPointing.Result()
        self.get_logger().info('Goal succeeded')
        return result
        # END of TODO 2


def main(args=None):
    rclpy.init(args=args)
    server = HeadActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()