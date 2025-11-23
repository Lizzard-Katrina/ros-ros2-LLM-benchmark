#!/usr/bin/env python3
import rospy
# TODO: import correct service type
from gazebo_msgs.srv import ________  # SetModelState
from gazebo_msgs.msg import ModelState

def set_model_state():
    # TODO: fill in the set_model_state service name
    rospy.wait_for_service("__________")
    
    try:
        # TODO: create ServiceProxy
        set_srv = ________
        
        # TODO: create ModelState object and set model name, pose, twist
        state = ________
        
        # TODO: call service and handle response
        resp = ________
        rospy.loginfo("Model state set successfully: ________")
    
    except Exception as e:
        rospy.logerr("Setting model state failed: ________")

if __name__ == "__main__":
    rospy.init_node("set_model_state_node")
    set_model_state()
