#!/usr/bin/env python
"""
Helper node that provides a mock /gazebo/set_model_state service
for runtime testing of SimMonitorState.

Works with or without the real gazebo_msgs package installed.
"""
import os
import sys
import time

# Ensure gazebo_msgs is importable (real or stub)
pkg_root = os.path.dirname(os.path.abspath(__file__))
if pkg_root not in sys.path:
    sys.path.insert(0, pkg_root)
import _ensure_gazebo_msgs
_ensure_gazebo_msgs.ensure()

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState


class MockGazeboService(Node):
    def __init__(self):
        super().__init__('mock_gazebo_service')
        self.srv = self.create_service(
            SetModelState,
            '/gazebo/set_model_state',
            self.handle_set_model_state)
        self.get_logger().info('Mock /gazebo/set_model_state service ready.')

    def handle_set_model_state(self, request, response):
        self.get_logger().info('Received set_model_state request for model: %s' % request.model_state.model_name)
        response.success = True
        response.status_message = 'Mock success'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockGazeboService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()