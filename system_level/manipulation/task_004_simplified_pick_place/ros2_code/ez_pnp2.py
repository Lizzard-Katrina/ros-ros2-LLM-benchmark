#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
import tf2_ros
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel
from ez_pick_and_place.srv import EzSceneSetup, EzStartPlanning
from moveit_msgs.srv import GraspPlanning, GetPositionIK

from ez_tools import EZToolSet

def main(args=None):
    moveit_commander.roscpp_initialize(sys.argv)
    rclpy.init(args=args)
    node = rclpy.create_node("ez_pnp")

    ez_tools = EZToolSet(node)

    # TODO
    # You must initialize the core system handles (EZToolSet) and establish 
    # persistent connections to the Grasping Engine and the Motion Planner.
    # 1. Ensure the system can 'listen' to spatial transforms (TF2).
    # 2. Bind the external Service APIs for 'Scene Setup' and 'Task Planning'.
    # 3. Secure the boot sequence: the node must block until all 4 dependent 
    #    backend services (GraspIt & IK) are verified to be in a 'Ready' state.
    # END OF TODO
    ez_tools.tf2_buffer = tf2_ros.Buffer()
    ez_tools.tf2_listener = tf2_ros.TransformListener(ez_tools.tf2_buffer, node)

    node.create_service(EzSceneSetup, '/ez_pnp/scene_setup', ez_tools.sceneSetup)
    node.create_service(EzStartPlanning, '/ez_pnp/start_planning', ez_tools.startPlanning)

    ez_tools.planning_srv = node.create_client(GraspPlanning, '/plan_grasps')
    ez_tools.add_model_srv = node.create_client(AddToDatabase, '/graspit_add_to_database')
    ez_tools.load_model_srv = node.create_client(LoadDatabaseModel, '/graspit_load_model')
    ez_tools.compute_ik_srv = node.create_client(GetPositionIK, '/compute_ik')

    node.get_logger().info("Waiting for backend services...")
    ez_tools.planning_srv.wait_for_service()
    ez_tools.add_model_srv.wait_for_service()
    ez_tools.load_model_srv.wait_for_service()
    ez_tools.compute_ik_srv.wait_for_service()
    node.get_logger().info("All backend services ready.")

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()