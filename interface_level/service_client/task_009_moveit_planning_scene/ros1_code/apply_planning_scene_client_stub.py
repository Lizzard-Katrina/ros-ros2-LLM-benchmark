#!/usr/bin/env python3
import rospy
# TODO: import correct service type
from moveit_msgs.srv import ________  # ApplyPlanningScene
from moveit_msgs.msg import PlanningScene

def apply_planning_scene_client():
    rospy.wait_for_service("/apply_planning_scene")
    
    try:
        # TODO: create ServiceProxy
        apply_scene_srv = ________
        
        # TODO: create PlanningScene diff object
        scene_diff = PlanningScene()
        # TODO: fill in robot_state, collision_objects, etc.
        # scene_diff.robot_state = ________
        # scene_diff.is_diff = True
        
        # TODO: call service
        resp = ________
        rospy.loginfo("Planning scene applied successfully")
    
    except Exception as e:
        rospy.logerr("Failed to apply planning scene: ________")

if __name__ == "__main__":
    rospy.init_node("apply_planning_scene_client_node")
    apply_planning_scene_client()
