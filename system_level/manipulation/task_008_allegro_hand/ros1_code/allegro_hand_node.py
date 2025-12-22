#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np

class AllegroHandNode:
    def __init__(self):
        rospy.init_node('allegro_hand_node')
        self.joint_pub = rospy.Publisher('allegro_hand/joint_states', JointState, queue_size=10)
        self.emergency_pub = rospy.Publisher('allegro_hand/emergency', Bool, queue_size=10)
        
        # Initialize Allegro Hand driver
        self.hand = self.init_hand_driver()
    #TODO: fill in the below 3 functions to Initialize Allegro Hand driver
    # Read current joint positions
    # Send torque commands to Allegro Hand
    def init_hand_driver(self):

    def read_joints(self):
        return positions

    def write_torque(self, torque):
    # END of TODO

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            joints = self.read_joints()
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = [f'joint_{i}' for i in range(16)]
            msg.position = joints.tolist()
            self.joint_pub.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    node = AllegroHandNode()
    node.run()
