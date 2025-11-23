#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import ________  # DeleteModel

def delete_model():
    # TODO: fill in the delete service name
    rospy.wait_for_service("__________")
    
    try:
        # TODO: create ServiceProxy
        delete_srv = ________
        
        # TODO: define model to delete
        model_name = "__________"
        
        # TODO: call service and handle response
        resp = ________
        rospy.loginfo("Model deleted successfully: ________")
        
    except Exception as e:
        rospy.logerr("Delete failed: ________")

if __name__ == "__main__":
    rospy.init_node("delete_model_node")
    delete_model()
