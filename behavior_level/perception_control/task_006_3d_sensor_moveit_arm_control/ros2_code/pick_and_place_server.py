#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from spherical_grasps_server import SphericalGrasps
from moveit_commander import PlanningSceneInterface
from moveit_msgs.action import Pickup, Place
from moveit_msgs.msg import Grasp, PlaceLocation, MoveItErrorCodes
from geometry_msgs.msg import Pose, PoseStamped
from tiago_pick_demo.action import PickUpPose
from moveit_msgs.srv import GetPlanningScene
from std_srvs.srv import Empty
from copy import deepcopy
from random import shuffle
import time

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def createPickupGoal(group="arm_torso", target="part",
                     grasp_pose=PoseStamped(),
                     possible_grasps=[],
                     links_to_allow_contact=None):
    """ Create a PickupGoal with the provided data"""
    pug = Pickup.Goal()
    pug.target_name = target
    pug.group_name = group
    pug.possible_grasps.extend(possible_grasps)
    pug.allowed_planning_time = 35.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 1  # 10
    pug.allowed_touch_objects = []
    pug.attached_object_touch_links = ['<octomap>']
    pug.attached_object_touch_links.extend(links_to_allow_contact)

    return pug


def createPlaceGoal(place_pose,
                    place_locations,
                    group="arm_torso",
                    target="part",
                    links_to_allow_contact=None):
    """Create PlaceGoal with the provided data"""
    placeg = Place.Goal()
    placeg.group_name = group
    placeg.attached_object_name = target
    placeg.place_locations = place_locations
    placeg.allowed_planning_time = 15.0
    placeg.planning_options.planning_scene_diff.is_diff = True
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
    placeg.planning_options.plan_only = False
    placeg.planning_options.replan = True
    placeg.planning_options.replan_attempts = 1
    placeg.allowed_touch_objects = ['<octomap>']
    placeg.allowed_touch_objects.extend(links_to_allow_contact)

    return placeg


class PickAndPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_and_place_server')
        self.get_logger().info("Initalizing PickAndPlaceServer...")
        self.sg = SphericalGrasps()
        self.get_logger().info("Connecting to pickup AS")
        self.pickup_ac = ActionClient(self, Pickup, '/pickup')
        self.pickup_ac.wait_for_server()
        self.get_logger().info("Succesfully connected.")
        self.get_logger().info("Connecting to place AS")
        self.place_ac = ActionClient(self, Place, '/place')
        self.place_ac.wait_for_server()
        self.get_logger().info("Succesfully connected.")
        self.scene = PlanningSceneInterface()
        self.get_logger().info("Connecting to /get_planning_scene service")
        self.scene_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        while not self.scene_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/get_planning_scene service not available, waiting...')
        self.get_logger().info("Connected.")

        self.get_logger().info("Connecting to clear octomap service...")
        self.clear_octomap_cli = self.create_client(Empty, '/clear_octomap')
        while not self.clear_octomap_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/clear_octomap service not available, waiting...')
        self.get_logger().info("Connected!")

        # Get the object size
        self.object_height = self.get_parameter_or('object_height', 0.0)
        self.object_width = self.get_parameter_or('object_width', 0.0)
        self.object_depth = self.get_parameter_or('object_depth', 0.0)

        # Get the links of the end effector exclude from collisions
        self.links_to_allow_contact = self.get_parameter_or('links_to_allow_contact', None)
        if self.links_to_allow_contact is None:
            self.get_logger().warn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
            self.links_to_allow_contact = []
        else:
            self.get_logger().info("Found links to allow contacts: " + str(self.links_to_allow_contact))

        self.pick_as = ActionServer(
            self,
            PickUpPose,
            '/pickup_pose',
            execute_callback=self.pick_cb)

        self.place_as = ActionServer(
            self,
            PickUpPose,
            '/place_pose',
            execute_callback=self.place_cb)

    def get_parameter_or(self, name, default):
        try:
            return self.get_parameter(name).value
        except Exception:
            return default

    async def pick_cb(self, goal_handle):
        """
        :type goal_handle: rclpy.action.server.GoalHandle
        """
        goal = goal_handle.request
        error_code = await self.grasp_object(goal.object_pose)
        result = PickUpPose.Result()
        result.error_code = error_code
        if error_code != MoveItErrorCodes.SUCCESS:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return result

    async def place_cb(self, goal_handle):
        """
        :type goal_handle: rclpy.action.server.GoalHandle
        """
        goal = goal_handle.request
        error_code = await self.place_object(goal.object_pose)
        result = PickUpPose.Result()
        result.error_code = error_code
        if error_code != MoveItErrorCodes.SUCCESS:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return result

    def wait_for_planning_scene_object(self, object_name='part'):
        self.get_logger().info(
            "Waiting for object '" + object_name + "' to appear in planning scene...")
        req = GetPlanningScene.Request()
        req.components.components = req.components.WORLD_OBJECT_NAMES

        while rclpy.ok():
            future = self.scene_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                planning_scene = future.result().scene
                world = planning_scene.world
                collision_objects = world.collision_objects
                for co in collision_objects:
                    if co.id == object_name:
                        self.get_logger().info("'" + object_name + "' is in scene!")
                        return
            time.sleep(0.5)

    async def grasp_object(self, object_pose):
        self.get_logger().info("Removing any previous 'part' object")
        # Remove previous object and add new object and support surface
        self.scene.remove_world_object('part')
        self.scene.remove_world_object('support')
        # Add support surface
        support_pose = PoseStamped()
        support_pose.header.frame_id = 'base_footprint'
        support_pose.pose.position.x = 0.7
        support_pose.pose.position.y = 0.0
        support_pose.pose.position.z = 0.0
        support_pose.pose.orientation.w = 1.0
        self.scene.add_box('support', support_pose, size=(0.8, 1.2, 0.01))
        # Add object
        obj_pose = deepcopy(object_pose)
        obj_pose.header.frame_id = 'base_footprint'
        self.scene.add_box('part', obj_pose, size=(self.object_width, self.object_depth, self.object_height))

        # Wait for scene update
        self.wait_for_planning_scene_object('part')

        # Generate grasps
        possible_grasps = self.sg.generate_grasps(obj_pose)

        # Shuffle grasps to add randomness
        shuffle(possible_grasps)

        # Create pickup goal
        pickup_goal = createPickupGoal(
            group="arm_torso",
            target="part",
            grasp_pose=obj_pose,
            possible_grasps=possible_grasps,
            links_to_allow_contact=self.links_to_allow_contact)

        # Send goal and wait for result
        send_goal_future = self.pickup_ac.send_goal_async(pickup_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pickup goal rejected')
            return MoveItErrorCodes.FAILURE
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        return result.error_code.val

    async def place_object(self, object_pose):
        self.get_logger().info("Clearing octomap")
        clear_req = Empty.Request()
        clear_future = self.clear_octomap_cli.call_async(clear_req)
        rclpy.spin_until_future_complete(self, clear_future)

        place_pose = deepcopy(object_pose)
        place_pose.header.frame_id = 'base_footprint'

        place_location = PlaceLocation()
        place_location.place_pose = place_pose
        place_location.post_place_posture.joint_names = []
        place_location.post_place_posture.points = []
        place_location.pre_place_approach.direction.header.frame_id = 'base_footprint'
        place_location.pre_place_approach.direction.vector.z = -1.0
        place_location.pre_place_approach.min_distance = 0.095
        place_location.pre_place_approach.desired_distance = 0.115
        place_location.post_place_retreat.direction.header.frame_id = 'base_footprint'
        place_location.post_place_retreat.direction.vector.z = 1.0
        place_location.post_place_retreat.min_distance = 0.1
        place_location.post_place_retreat.desired_distance = 0.25

        place_locations = [place_location]

        # Try with primary group
        place_goal = createPlaceGoal(
            place_pose=place_pose,
            place_locations=place_locations,
            group="arm_torso",
            target="part",
            links_to_allow_contact=self.links_to_allow_contact)

        send_goal_future = self.place_ac.send_goal_async(place_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Place goal rejected')
            return MoveItErrorCodes.FAILURE
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn("Place with 'arm_torso' group failed, retrying with 'arm' group")
            place_goal = createPlaceGoal(
                place_pose=place_pose,
                place_locations=place_locations,
                group="arm",
                target="part",
                links_to_allow_contact=self.links_to_allow_contact)

            send_goal_future = self.place_ac.send_goal_async(place_goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Place goal rejected')
                return MoveItErrorCodes.FAILURE
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result().result

        return result.error_code.val


def main(args=None):
    rclpy.init(args=args)
    paps = PickAndPlaceServer()
    rclpy.spin(paps)
    paps.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()