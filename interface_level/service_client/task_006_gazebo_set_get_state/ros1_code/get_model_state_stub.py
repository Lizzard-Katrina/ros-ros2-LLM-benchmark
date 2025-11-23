#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import ________  # GetModelState

def get_model_state():
    # TODO: fill in the get_model_state service name
    rospy.wait_for_service("__________")
    
    try:
        # TODO: create ServiceProxy
        get_srv = ________
        
        # TODO: define model name to query
        model_name = "__________"
        
        # TODO: call service and handle response
        resp = ________
        rospy.loginfo("Model state retrieved: ________")
    
    except Exception as e:
        rospy.logerr("Getting model state failed: ________")

if __name__ == "__main__":
    rospy.init_node("get_model_state_node")
    get_model_state()
