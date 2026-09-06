#!/usr/bin/env python3
"""
Mock service node that provides /gazebo/set_model_state service.
Writes received request details to a temp file for verification.
"""
import sys
import os
import json
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState


class MockSetModelStateService(Node):
    def __init__(self, output_file):
        super().__init__('mock_set_model_state_service')
        self.output_file = output_file
        self.srv = self.create_service(
            SetModelState, '/gazebo/set_model_state', self.handle_request
        )
        self.get_logger().info('Mock /gazebo/set_model_state service is ready.')

    def handle_request(self, request, response):
        self.get_logger().info(f'Received request for model: {request.model_state.model_name}')
        data = {
            'model_name': request.model_state.model_name,
            'reference_frame': request.model_state.reference_frame,
            'pose': {
                'position': {
                    'x': request.model_state.pose.position.x,
                    'y': request.model_state.pose.position.y,
                    'z': request.model_state.pose.position.z,
                },
                'orientation': {
                    'x': request.model_state.pose.orientation.x,
                    'y': request.model_state.pose.orientation.y,
                    'z': request.model_state.pose.orientation.z,
                    'w': request.model_state.pose.orientation.w,
                },
            },
            'twist': {
                'linear': {
                    'x': request.model_state.twist.linear.x,
                    'y': request.model_state.twist.linear.y,
                    'z': request.model_state.twist.linear.z,
                },
                'angular': {
                    'x': request.model_state.twist.angular.x,
                    'y': request.model_state.twist.angular.y,
                    'z': request.model_state.twist.angular.z,
                },
            },
        }
        with open(self.output_file, 'w') as f:
            json.dump(data, f)
        response.success = True
        response.status_message = 'OK'
        return response


def main():
    output_file = sys.argv[1] if len(sys.argv) > 1 else '/tmp/mock_service_request.json'
    rclpy.init()
    node = MockSetModelStateService(output_file)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()