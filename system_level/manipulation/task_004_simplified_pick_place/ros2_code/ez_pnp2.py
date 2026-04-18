#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
import tf2_ros
from moveit_commander import MoveGroupCommander, PlanningScene
from geometry_msgs.msg import PoseStamped
from ez_pick_and_place.srv import EzSceneSetup, EzStartPlanning
from moveit_msgs.srv import GraspPlanning, GetPositionIK

from ez_tools import EZToolSet

class EzPnpNode(Node):
    def __init__(self):
        super().__init__('ez_pnp')
        self.ez_tools = EZToolSet(self)

        # Initialize the core system handles (EZToolSet) and establish 
        # persistent connections to the Grasping Engine and the Motion Planner.
        self.ez_tools.tf2_buffer = tf2_ros.Buffer()
        self.ez_tools.tf2_listener = tf2_ros.TransformListener(self.ez_tools.tf2_buffer)
        self.ez_tools.moveit_scene = PlanningScene()
        self.ez_tools.planning_srv = self.create_client(GraspPlanning, 'grasp_planning')
        self.ez_tools.add_model_srv = self.create_client(AddToDatabase, 'add_to_database')
        self.ez_tools.load_model_srv = self.create_client(LoadDatabaseModel, 'load_database_model')
        self.ez_tools.compute_ik_srv = self.create_client(GetPositionIK, 'compute_ik')

        # Bind the external Service APIs for 'Scene Setup' and 'Task Planning'.
        self.scene_setup_srv = self.create_service(EzSceneSetup, 'scene_setup', self.ez_tools.sceneSetup)
        self.start_planning_srv = self.create_service(EzStartPlanning, 'start_planning', self.ez_tools.startPlanning)

        # Secure the boot sequence: the node must block until all 4 dependent 
        # backend services (GraspIt & IK) are verified to be in a 'Ready' state.
        while not self.ez_tools.planning_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Grasp planning service not available, waiting...')
        while not self.ez_tools.add_model_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Add to database service not available, waiting...')
        while not self.ez_tools.load_model_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Load database model service not available, waiting...')
        while not self.ez_tools.compute_ik_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Compute IK service not available, waiting...')

def main(args=None):
    rclpy.init(args=args)
    ez_pnp_node = EzPnpNode()
    rclpy.spin(ez_pnp_node)
    ez_pnp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()