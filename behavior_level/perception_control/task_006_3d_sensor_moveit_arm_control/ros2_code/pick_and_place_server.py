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

import time
import copy
from copy import deepcopy
from random import shuffle

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor

from spherical_grasps_server import SphericalGrasps
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, MoveItErrorCodes, PlaceLocation
from moveit_msgs.action import Pickup, Place
from moveit_msgs.srv import GetPlanningScene
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.action import PickUpPose
from std_srvs.srv import Empty

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
    if links_to_allow_contact is None:
        links_to_allow_contact = []

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
    if links_to_allow_contact is None:
        links_to_allow_contact = []

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
        self.scene_srv = self.create_client(GetPlanningScene, '/get_planning_scene')
        while not self.scene_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /get_planning_scene service...")
        self.get_logger().info("Connected.")

        self.get_logger().info("Connecting to clear octomap service...")
        self.clear_octomap_srv = self.create_client(Empty, '/clear_octomap')
        while not self.clear_octomap_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /clear_octomap service...")
        self.get_logger().info("Connected!")

        # Get the object size
        self.declare_parameter('object_height', 0.1)
        self.declare_parameter('object_width', 0.03)
        self.declare_parameter('object_depth', 0.03)
        self.declare_parameter('links_to_allow_contact', [])

        self.object_height = float(self.get_parameter('object_height').value)
        self.object_width = float(self.get_parameter('object_width').value)
        self.object_depth = float(self.get_parameter('object_depth').value)

        # Get the links of the end effector exclude from collisions
        self.links_to_allow_contact = self.get_parameter('links_to_allow_contact').value
        if self.links_to_allow_contact is None or len(self.links_to_allow_contact) == 0:
            self.links_to_allow_contact = []
            self.get_logger().warn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
        else:
            self.get_logger().info("Found links to allow contacts: " + str(self.links_to_allow_contact))

        self.pick_as = ActionServer(
            self,
            PickUpPose,
            '/pickup_pose',
            execute_callback=self.pick_cb
        )

        self.place_as = ActionServer(
            self,
            PickUpPose,
            '/place_pose',
            execute_callback=self.place_cb
        )

    def pick_cb(self, goal_handle):
        """
        :type goal: PickUpPose.Goal
        """
        goal = goal_handle.request
        error_code = self.grasp_object(goal.object_pose)
        p_res = PickUpPose.Result()
        p_res.error_code = int(error_code)
        if error_code != MoveItErrorCodes.SUCCESS:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return p_res

    def place_cb(self, goal_handle):
        """
        :type goal: PickUpPose.Goal
        """
        goal = goal_handle.request
        error_code = self.place_object(goal.object_pose)
        p_res = PickUpPose.Result()
        p_res.error_code = int(error_code)
        if error_code != MoveItErrorCodes.SUCCESS:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return p_res

    def wait_for_planning_scene_object(self, object_name='part'):
        self.get_logger().info(
            "Waiting for object '" + object_name + "'' to appear in planning scene...")
        gps_req = GetPlanningScene.Request()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES

        found = False
        while rclpy.ok() and not found:
            future = self.scene_srv.call_async(gps_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                scene = future.result().scene
                for co in scene.world.collision_objects:
                    if co.id == object_name:
                        found = True
                        break
            if not found:
                time.sleep(0.1)

        self.get_logger().info("'" + object_name + "'' is in scene!")

    def grasp_object(self, object_pose):
        self.get_logger().info("Removing any previous 'part' object")
        self.scene.remove_world_object('part')
        self.scene.remove_world_object('table')
        time.sleep(0.5)

        self.get_logger().info("Adding new 'part' object")
        self.scene.add_box(
            'part',
            object_pose,
            size=(self.object_depth, self.object_width, self.object_height)
        )

        self.get_logger().info("Adding supporting surface 'table'")
        table_pose = deepcopy(object_pose)
        table_height = max(object_pose.pose.position.z - (self.object_height / 2.0), 0.01)
        table_pose.pose.position.z = table_height / 2.0
        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.0
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box(
            'table',
            table_pose,
            size=(1.5, 1.5, table_height)
        )

        self.wait_for_planning_scene_object('part')
        self.wait_for_planning_scene_object('table')

        self.get_logger().info("Clearing octomap")
        clear_future = self.clear_octomap_srv.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, clear_future, timeout_sec=5.0)

        grasps = self.sg.create_grasps_from_object_pose(object_pose)
        shuffle(grasps)

        pug = createPickupGoal(
            group="arm_torso",
            target="part",
            grasp_pose=object_pose,
            possible_grasps=grasps,
            links_to_allow_contact=self.links_to_allow_contact
        )
        pug.support_surface_name = 'table'

        send_goal_future = self.pickup_ac.send_goal_async(pug)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Pickup goal was rejected")
            return MoveItErrorCodes.FAILURE

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return MoveItErrorCodes.FAILURE

        result = wrapped_result.result
        return result.error_code.val

    def place_object(self, object_pose):
        self.get_logger().info("Clearing octomap")
        clear_future = self.clear_octomap_srv.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, clear_future, timeout_sec=5.0)

        place_locations = self.sg.create_placings_from_object_pose(object_pose)
        shuffle(place_locations)

        last_error = MoveItErrorCodes.FAILURE
        for group_name in ["arm_torso", "arm"]:
            self.get_logger().info(f"Trying place with group '{group_name}'")
            placeg = createPlaceGoal(
                object_pose,
                copy.deepcopy(place_locations),
                group=group_name,
                target="part",
                links_to_allow_contact=self.links_to_allow_contact
            )

            send_goal_future = self.place_ac.send_goal_async(placeg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warn(f"Place goal rejected for group '{group_name}'")
                last_error = MoveItErrorCodes.FAILURE
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            wrapped_result = result_future.result()
            if wrapped_result is None:
                last_error = MoveItErrorCodes.FAILURE
                continue

            result = wrapped_result.result
            last_error = result.error_code.val
            if last_error == MoveItErrorCodes.SUCCESS:
                return last_error

            self.get_logger().warn(
                f"Place failed with group '{group_name}' and error "
                f"{moveit_error_dict.get(last_error, str(last_error))}, retrying if possible..."
            )

        return last_error


def main(args=None):
    rclpy.init(args=args)
    paps = PickAndPlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(paps)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        paps.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()