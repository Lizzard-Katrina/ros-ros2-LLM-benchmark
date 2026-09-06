#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from task_003_ur5_red_box.srv import SetJointStates
from std_msgs.msg import Float32


class SetJointStatesClient(Node):
    def __init__(self):
        super().__init__('set_joint_states_client')
        self.cli = self.create_client(SetJointStates, 'set_joint_states')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for set_joint_states service...')

    def send_request(self, joint_states):
        request = SetJointStates.Request()
        request.forearm_0 = Float32(data=joint_states[0])
        request.forearm_1 = Float32(data=joint_states[1])
        request.arm_0 = Float32(data=joint_states[2])
        request.arm_1 = Float32(data=joint_states[3])
        future = self.cli.call_async(request)
        return future


def set_joint_states(joint_states):
    rclpy.init()
    node = SetJointStatesClient()
    try:
        future = node.send_request(joint_states)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        result = future.result()
        if result is not None:
            node.get_logger().info(f'Result: success={result.success}, message={result.message}')
        else:
            node.get_logger().error('Service call failed or timed out.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) == 5:
        joint_states = [
            float(sys.argv[1]),
            float(sys.argv[2]),
            float(sys.argv[3]),
            float(sys.argv[4]),
        ]
        set_joint_states(joint_states)
    else:
        print('Not enough arguments. Four arguments required: forearm_0, forearm_1, arm_0, arm_1')
        sys.exit(1)