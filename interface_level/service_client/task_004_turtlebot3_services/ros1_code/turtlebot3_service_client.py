#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty

def call_take_pano():
    rospy.wait_for_service("__________")  # TODO: correct service name
    try:
        take_pano = rospy.ServiceProxy("__________", Empty)  # TODO
        take_pano()
        rospy.loginfo("Panorama request sent.")
    except Exception as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node("turtlebot3_pano_client")
    call_take_pano()
