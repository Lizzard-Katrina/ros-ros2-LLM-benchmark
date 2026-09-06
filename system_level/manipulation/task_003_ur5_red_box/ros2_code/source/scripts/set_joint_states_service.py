#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from task_003_ur5_red_box.srv import SetJointStates


class SetJointStatesServer(Node):
    def __init__(self):
        super().__init__('set_joint_states_server')
        self.srv = self.create_service(
            SetJointStates,
            'set_joint_states',
            self.set_joint_states_callback
        )
        self.get_logger().info('SetJointStates service server is ready.')

    def set_joint_states_callback(self, request, response):
        self.get_logger().info('Received joint state request.')
        forearm_0 = request.forearm_0.data
        forearm_1 = request.forearm_1.data
        arm_0 = request.arm_0.data
        arm_1 = request.arm_1.data

        self.get_logger().info(
            f'Joint goals: [{forearm_0}, {forearm_1}, {arm_0}, {arm_1}]'
        )

        # In a full system this would command MoveIt to move the robot.
        # Here we acknowledge the request.
        response.success = True
        response.message = (
            f'Joint states set to [{forearm_0}, {forearm_1}, {arm_0}, {arm_1}]'
        )
        return response


def set_joint_states_server():
    rclpy.init(args=sys.argv)
    node = SetJointStatesServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    set_joint_states_server()