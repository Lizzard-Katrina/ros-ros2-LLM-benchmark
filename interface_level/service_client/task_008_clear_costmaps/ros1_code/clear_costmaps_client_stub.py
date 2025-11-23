#!/usr/bin/env python3
import rospy
# TODO: import correct service type
from std_srvs.srv import ________  # Empty service

def clear_costmaps_client():
    # TODO: fill in service name
    rospy.wait_for_service("/move_base/clear_costmaps")
    
    try:
        # TODO: create ServiceProxy
        clear_srv = ________
        
        # TODO: call service and handle response
        resp = ________
        rospy.loginfo("Costmaps cleared successfully")
    
    except Exception as e:
        rospy.logerr("Failed to clear costmaps: ________")

if __name__ == "__main__":
    rospy.init_node("clear_costmaps_client_node")
    clear_costmaps_client()
