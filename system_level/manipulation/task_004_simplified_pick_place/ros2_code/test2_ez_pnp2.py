#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from ez_pick_and_place.srv import EzSceneSetup, EzStartPlanning
from ez_pick_and_place.msg import EzModel
from geometry_msgs.msg import PoseStamped

# Note:
# In order to run this test you need the roboskel_ros_resources package!

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("ez_graspit")
    
    scene_setup_srv = node.create_client(EzSceneSetup, "/ez_pnp/scene_setup")
    print("Waiting for the services to come up...")
    scene_setup_srv.wait_for_service()
    
    start_planning_srv = node.create_client(EzStartPlanning, "/ez_pnp/start_planning")
    start_planning_srv.wait_for_service()
    print("Done!")

    # [TODO]: INTEGRATION_SCENARIO_DEFINITION
    # Define a complete test case involving:
    # 1. A table obstacle and two objects ('E' and 'Z') with their respective GraspIt/MoveIt files.
    # 2. A specific 'gripper_frame' to anchor the planning coordinate system.
    # 3. A planning request to move object 'Z' to a designated target pose in 'world' frame.
    # END OF TODO
    setup_req = EzSceneSetup.Request()
    setup_req.gripper_frame = "base_link"
    
    obj_z = EzModel()
    obj_z.name = "Z"
    obj_z.graspit_file = "models/objects/z.xml"
    obj_z.moveit_file = "package://roboskel_ros_resources/models/z.stl"
    obj_z.pose.header.frame_id = "world"
    
    obj_e = EzModel()
    obj_e.name = "E"
    obj_e.graspit_file = "models/objects/e.xml"
    obj_e.moveit_file = "package://roboskel_ros_resources/models/e.stl"
    obj_e.pose.header.frame_id = "world"
    
    table = EzModel()
    table.name = "table"
    table.graspit_file = "models/obstacles/table.xml"
    table.moveit_file = "package://roboskel_ros_resources/models/table.stl"
    table.pose.header.frame_id = "world"
    
    setup_req.objects = [obj_z, obj_e]
    setup_req.obstacles = [table]
    
    scene_setup_srv.call_async(setup_req)
    
    plan_req = EzStartPlanning.Request()
    plan_req.graspit_target_object = "Z"
    plan_req.arm_move_group = "arm"
    plan_req.gripper_move_group = "gripper"
    
    target_pose = PoseStamped()
    target_pose.header.frame_id = "world"
    target_pose.pose.position.x = 0.5
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.w = 1.0
    plan_req.target_place = target_pose
    
    future = start_planning_srv.call_async(plan_req)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    print(response)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()