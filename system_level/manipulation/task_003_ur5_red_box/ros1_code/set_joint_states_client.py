#!/usr/bin/env python

import sys
import rospy
from robotic_arm_algorithms.srv import *

def set_joint_states(joint_states):
# TODO: Implement a robust ROS 2 service client. 
# The client must handle the service call asynchronously to prevent 
# blocking the main executor, and it should gracefully wait for 
# the response before exiting.
# END OF TODO

if __name__ == "__main__":
    if len(sys.argv) == 5:
        joint_states = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]
        set_joint_states(joint_states)
    else:
        print("not enaugh argument. Four arguments required: forearm 0, forearm 1, arm 0, arm 1")
        sys.exit(1)
        
