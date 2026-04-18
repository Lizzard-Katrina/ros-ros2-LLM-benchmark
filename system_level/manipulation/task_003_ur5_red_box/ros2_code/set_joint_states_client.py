#!/usr/bin/env python

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from robotic_arm_algorithms.srv import SetJointStates

class SetJointStatesClient(Node):
    def __init__(self):
        super().__init__('set_joint_states_client')
        self.cli = self.create_client(SetJointStates, 'set_joint_states')

    def send_request(self, joint_states):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        req = SetJointStates.Request()
        req.forearm_0 = joint_states[0]
        req.forearm_1 = joint_states[1]
        req.arm_0 = joint_states[2]
        req.arm_1 = joint_states[3]
        self.future = self.cli.call_async(req)
        return self.future

def set_joint_states(joint_states):
    rclpy.init()
    client = SetJointStatesClient()
    future = client.send_request(joint_states)
    while rclpy.ok():
        rclpy.spin_once(client)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info('Result: %s' % response.success)
            break
    client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    if len(sys.argv) == 5:
        joint_states = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]
        set_joint_states(joint_states)
    else:
        print("not enough argument. Four arguments required: forearm 0, forearm 1, arm 0, arm 1")
        sys.exit(1)