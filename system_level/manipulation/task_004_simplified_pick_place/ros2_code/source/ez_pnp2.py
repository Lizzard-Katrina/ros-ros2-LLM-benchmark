#!/usr/bin/env python3
import sys
import rclpy
import tf2_ros

from task_004_simplified_pick_place.ez_tools import EZToolSet


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('ez_pnp')

    ez_tools = EZToolSet()
    ez_tools.node = node

    ez_tools.debug = node.declare_parameter('debug', False).get_parameter_value().bool_value

    # Initialize TF2 buffer and listener using tf2_ros.Buffer()
    ez_tools.tf2_buffer = tf2_ros.Buffer()
    ez_tools.tf2_listener = tf2_ros.TransformListener(ez_tools.tf2_buffer, node)

    try:
        from moveit_msgs.srv import GraspPlanning, GetPositionIK

        # Create service clients (create_client) for GraspIt and IK
        ez_tools.add_model_srv = node.create_client(
            GraspPlanning, '/graspit_add_to_database')
        ez_tools.load_model_srv = node.create_client(
            GraspPlanning, '/graspit_load_model')
        ez_tools.planning_srv = node.create_client(
            GraspPlanning, '/graspit_eg_planning')
        ez_tools.compute_ik_srv = node.create_client(
            GetPositionIK, '/compute_ik')

        # Create services for scene setup and start planning
        node.create_service(
            GraspPlanning, 'ez_pnp/scene_setup', ez_tools.sceneSetup)
        node.create_service(
            GraspPlanning, 'ez_pnp/start_planning', ez_tools.startPlanning)

        # Wait for all 4 dependent backend services to be ready
        node.get_logger().info('Waiting for backend services...')
        ez_tools.add_model_srv.wait_for_service(timeout_sec=5.0)
        ez_tools.load_model_srv.wait_for_service(timeout_sec=5.0)
        ez_tools.planning_srv.wait_for_service(timeout_sec=5.0)
        ez_tools.compute_ik_srv.wait_for_service(timeout_sec=5.0)
        node.get_logger().info('All backend services ready.')
    except ImportError:
        node.get_logger().warn('moveit_msgs not available, running in limited mode')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()