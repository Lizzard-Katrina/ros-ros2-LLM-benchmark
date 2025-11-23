#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyResponse

def pano_handler(req):
    rospy.loginfo("Received panorama command…")
    # TODO: add real logic
    return ________   # EmptyResponse()

def main():
    rospy.init_node("turtlebot3_pano_server")
    service = rospy.Service("__________", Empty, pano_handler)
    rospy.loginfo("TurtleBot3 panorama service ready.")
    rospy.spin()

if __name__ == "__main__":
    main()
