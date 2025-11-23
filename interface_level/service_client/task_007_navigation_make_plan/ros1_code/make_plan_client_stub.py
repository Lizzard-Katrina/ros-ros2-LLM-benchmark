#!/usr/bin/env python3
import rospy
# TODO: import correct service type
from move_base_msgs.srv import ________  # GetPlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def make_plan_client():
    rospy.wait_for_service("/move_base/make_plan")
    
    try:
        # TODO: create ServiceProxy
        make_plan_srv = ________
        
        # TODO: define start and goal PoseStamped
        start = PoseStamped()
        start.header.frame_id = "__________"
        start.pose.position.x = ________
        start.pose.position.y = ________
        start.pose.orientation.w = ________
        
        goal = PoseStamped()
        goal.header.frame_id = "__________"
        goal.pose.position.x = ________
        goal.pose.position.y = ________
        goal.pose.orientation.w = ________
        
        # TODO: call service and handle response
        resp = ________
        rospy.loginfo("Received path with {} poses".format(len(resp.plan.poses)))
    
    except Exception as e:
        rospy.logerr("make_plan service call failed: ________")

if __name__ == "__main__":
    rospy.init_node("make_plan_client_node")
    make_plan_client()
