#!/usr/bin/env python
import sys
import rospy
import tf2_ros
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel
from ez_pick_and_place.srv import EzSceneSetup, EzStartPlanning
from moveit_msgs.srv import GraspPlanning, GetPositionIK

from ez_tools import EZToolSet

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ez_pnp")

    ez_tools = EZToolSet()

    # TODO
    # You must initialize the core system handles (EZToolSet) and establish 
    # persistent connections to the Grasping Engine and the Motion Planner.
    # 1. Ensure the system can 'listen' to spatial transforms (TF2).
    # 2. Bind the external Service APIs for 'Scene Setup' and 'Task Planning'.
    # 3. Secure the boot sequence: the node must block until all 4 dependent 
    #    backend services (GraspIt & IK) are verified to be in a 'Ready' state.
    # END OF TODO
    rospy.spin()

main()
