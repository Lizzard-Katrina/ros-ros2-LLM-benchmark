#!/usr/bin/env python

import sys
import rclpy
from rclpy.node import Node
from robotic_arm_algorithms.srv import SetJointStates

def set_joint_states(joint_states):
    rclpy.init()
    node = rclpy.create_node('set_joint_states_client')
    client = node.create_client(SetJointStates, 'set_joint_states')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
        
    req = SetJointStates.Request()
    req.forearm_0.data = joint_states[0]
    req.forearm_1.data = joint_states[1]
    req.arm_0.data = joint_states[2]
    req.arm_1.data = joint_states[3]
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Successfully set joint states')
    else:
        node.get_logger().error('Exception while calling service: %r' % future.exception())
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    if len(sys.argv) == 5:
        joint_states = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]
        set_joint_states(joint_states)
    else:
        print("not enaugh argument. Four arguments required: forearm 0, forearm 1, arm 0, arm 1")
        sys.exit(1)