#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    node = rclpy.create_node('ez_graspit')

    node.get_logger().info('Waiting for the services to come up...')

    class SceneSetupReq:
        pass

    class EzModel:
        def __init__(self):
            self.name = ""
            self.graspit_file = ""
            self.moveit_file = ""
            self.pose = PoseStamped()

    class GripperModel:
        def __init__(self):
            self.name = ""
            self.graspit_file = ""

    scene_req = SceneSetupReq()
    scene_req.pose_factor = 1000

    # Table obstacle
    table = EzModel()
    table.name = "table"
    table.graspit_file = "table.xml"
    table.moveit_file = "table.stl"
    table.pose = PoseStamped()
    table.pose.header.frame_id = "world"
    table.pose.pose.position.x = 0.0
    table.pose.pose.position.y = 0.0
    table.pose.pose.position.z = 0.0
    table.pose.pose.orientation.w = 1.0

    # Object E
    obj_e = EzModel()
    obj_e.name = "E"
    obj_e.graspit_file = "E.xml"
    obj_e.moveit_file = "E.stl"
    obj_e.pose = PoseStamped()
    obj_e.pose.header.frame_id = "world"
    obj_e.pose.pose.position.x = 0.5
    obj_e.pose.pose.position.y = 0.0
    obj_e.pose.pose.position.z = 0.75
    obj_e.pose.pose.orientation.w = 1.0

    # Object Z
    obj_z = EzModel()
    obj_z.name = "Z"
    obj_z.graspit_file = "Z.xml"
    obj_z.moveit_file = "Z.stl"
    obj_z.pose = PoseStamped()
    obj_z.pose.header.frame_id = "world"
    obj_z.pose.pose.position.x = 0.3
    obj_z.pose.pose.position.y = 0.2
    obj_z.pose.pose.position.z = 0.75
    obj_z.pose.pose.orientation.w = 1.0

    scene_req.objects = [obj_e, obj_z]
    scene_req.obstacles = [table]

    gripper = GripperModel()
    gripper.name = "gripper"
    gripper.graspit_file = "gripper.xml"
    scene_req.gripper = gripper
    scene_req.gripper_frame = "gripper_link"
    scene_req.finger_joint_names = ["finger_joint1", "finger_joint2"]

    class PlanReq:
        pass

    plan_req = PlanReq()
    plan_req.graspit_target_object = "Z"
    plan_req.arm_move_group = "arm"
    plan_req.gripper_move_group = "gripper"
    plan_req.max_replanning = 3

    target_place = PoseStamped()
    target_place.header.frame_id = "world"
    target_place.pose.position.x = -0.3
    target_place.pose.position.y = 0.2
    target_place.pose.position.z = 0.75
    target_place.pose.orientation.w = 1.0
    plan_req.target_place = target_place

    node.get_logger().info(f'Planning request: target={plan_req.graspit_target_object}, '
                           f'arm_group={plan_req.arm_move_group}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()