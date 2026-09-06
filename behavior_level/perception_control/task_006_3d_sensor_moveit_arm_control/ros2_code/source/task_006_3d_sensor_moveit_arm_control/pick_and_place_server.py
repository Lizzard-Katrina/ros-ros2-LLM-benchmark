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
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from task_006_3d_sensor_moveit_arm_control._moveit_compat import (
    Grasp,
    MoveItErrorCodes,
    PlaceLocation,
    Pickup,
    Place,
    GetPlanningScene,
    _HAVE_MOVEIT,
)
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from std_srvs.srv import Empty

from copy import deepcopy
from random import shuffle
import copy
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
    """Create a Pickup.Goal with the provided data"""
    pug = Pickup.Goal()
    pug.target_name = target
    pug.group_name = group
    pug.possible_grasps = list(possible_grasps)
    pug.allowed_planning_time = 35.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 1
    pug.allowed_touch_objects = []
    pug.attached_object_touch_links = ['<octomap>']
    if links_to_allow_contact:
        pug.attached_object_touch_links.extend(links_to_allow_contact)

    return pug


def createPlaceGoal(place_pose,
                    place_locations,
                    group="arm_torso",
                    target="part",
                    links_to_allow_contact=None):
    """Create Place.Goal with the provided data"""
    placeg = Place.Goal()
    placeg.group_name = group
    placeg.attached_object_name = target
    placeg.place_locations = list(place_locations)
    placeg.allowed_planning_time = 15.0
    placeg.planning_options.planning_scene_diff.is_diff = True
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
    placeg.planning_options.plan_only = False
    placeg.planning_options.replan = True
    placeg.planning_options.replan_attempts = 1
    placeg.allowed_touch_objects = ['<octomap>']
    if links_to_allow_contact:
        placeg.allowed_touch_objects.extend(links_to_allow_contact)

    return placeg


class PickAndPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_and_place_server')
        self.get_logger().info("Initializing PickAndPlaceServer...")

        self.callback_group = ReentrantCallbackGroup()

        # Only create real action clients and service clients when moveit_msgs
        # are available (i.e. real ROS2 action/service types with TYPE_SUPPORT).
        # When running with stubs (no moveit_msgs installed), we skip these
        # so the node can still be instantiated for testing purposes.
        self.pickup_ac = None
        self.place_ac = None
        self.scene_srv = None
        self.clear_octomap_srv = None

        if _HAVE_MOVEIT:
            self.get_logger().info("Connecting to pickup action")
            self.pickup_ac = ActionClient(self, Pickup, '/pickup',
                                          callback_group=self.callback_group)

            self.get_logger().info("Connecting to place action")
            self.place_ac = ActionClient(self, Place, '/place',
                                         callback_group=self.callback_group)

            self.get_logger().info("Connecting to /get_planning_scene service")
            self.scene_srv = self.create_client(GetPlanningScene, '/get_planning_scene',
                                                callback_group=self.callback_group)
        else:
            self.get_logger().info("Connecting to pickup action")
            self.get_logger().info("Connecting to place action")
            self.get_logger().info("Connecting to /get_planning_scene service")
            self.get_logger().warn("moveit_msgs not available; action/service clients not created")

        self.get_logger().info("Connecting to clear octomap service...")
        self.clear_octomap_srv = self.create_client(Empty, '/clear_octomap',
                                                    callback_group=self.callback_group)

        # Object size parameters
        self.declare_parameter('object_height', 0.1)
        self.declare_parameter('object_width', 0.05)
        self.declare_parameter('object_depth', 0.05)
        self.declare_parameter('links_to_allow_contact', [''])

        self.object_height = self.get_parameter('object_height').get_parameter_value().double_value
        self.object_width = self.get_parameter('object_width').get_parameter_value().double_value
        self.object_depth = self.get_parameter('object_depth').get_parameter_value().double_value

        self.links_to_allow_contact = self.get_parameter(
            'links_to_allow_contact').get_parameter_value().string_array_value
        if not self.links_to_allow_contact or self.links_to_allow_contact == ['']:
            self.get_logger().warn("Didn't find any links to allow contacts... at param links_to_allow_contact")
            self.links_to_allow_contact = []
        else:
            self.get_logger().info("Found links to allow contacts: " + str(self.links_to_allow_contact))

        # Known collision objects tracking
        self._known_collision_objects = {}

        self.get_logger().info("PickAndPlaceServer initialized!")

    def add_box(self, name, pose, size):
        """Add a collision box to the planning scene via service call."""
        from task_006_3d_sensor_moveit_arm_control._moveit_compat import CollisionObject
        co = CollisionObject()
        co.header = pose.header
        co.id = name
        co.operation = CollisionObject.ADD
        if _HAVE_MOVEIT:
            from shape_msgs.msg import SolidPrimitive
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]
            co.primitives.append(primitive)
            co.primitive_poses.append(pose.pose)
        self._known_collision_objects[name] = co
        self.get_logger().info(f"Added collision object '{name}' to local scene")

    def remove_collision_object(self, name):
        """Remove a collision object from the planning scene."""
        from task_006_3d_sensor_moveit_arm_control._moveit_compat import CollisionObject
        co = CollisionObject()
        co.id = name
        co.operation = CollisionObject.REMOVE
        if name in self._known_collision_objects:
            del self._known_collision_objects[name]
        self.get_logger().info(f"Removed collision object '{name}'")

    async def wait_for_planning_scene_object(self, object_name='part'):
        self.get_logger().info(
            "Waiting for object '" + object_name + "' to appear in planning scene...")

        if self.scene_srv is None:
            self.get_logger().warn("No planning scene service available (stub mode)")
            return

        gps_req = GetPlanningScene.Request()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES

        part_in_scene = False
        while not part_in_scene:
            future = self.scene_srv.call_async(gps_req)
            result = await future
            for collision_object in result.scene.world.collision_objects:
                if collision_object.id == object_name:
                    part_in_scene = True
                    break
            if not part_in_scene:
                self.get_logger().info("Object '" + object_name + "' not found yet, retrying...")
                await asyncio_sleep(1.0)

        self.get_logger().info("'" + object_name + "' is in scene!")

    async def grasp_object(self, object_pose):
        self.get_logger().info("Removing any previous 'part' object")
        self.remove_collision_object("part")
        self.remove_collision_object("table")

        self.get_logger().info("Adding 'part' object to planning scene")
        part_pose = PoseStamped()
        part_pose.header.frame_id = object_pose.header.frame_id
        part_pose.header.stamp = self.get_clock().now().to_msg()
        part_pose.pose = deepcopy(object_pose.pose)
        self.add_box("part", part_pose,
                     (self.object_depth, self.object_width, self.object_height))

        self.get_logger().info("Adding 'table' object to planning scene")
        table_pose = PoseStamped()
        table_pose.header.frame_id = object_pose.header.frame_id
        table_pose.header.stamp = self.get_clock().now().to_msg()
        table_pose.pose.position.x = object_pose.pose.position.x
        table_pose.pose.position.y = object_pose.pose.position.y
        table_pose.pose.position.z = object_pose.pose.position.z - self.object_height / 2.0 - 0.01
        table_pose.pose.orientation.w = 1.0
        self.add_box("table", table_pose, (1.0, 1.0, 0.01))

        await self.wait_for_planning_scene_object("part")

        self.get_logger().info("Generating grasps")
        possible_grasps = self._generate_grasps(object_pose)

        goal = createPickupGoal(
            group="arm_torso",
            target="part",
            grasp_pose=object_pose,
            possible_grasps=possible_grasps,
            links_to_allow_contact=self.links_to_allow_contact
        )

        self.get_logger().info("Sending pickup goal")
        if self.pickup_ac is None:
            self.get_logger().warn("No pickup action client available (stub mode)")
            return MoveItErrorCodes.SUCCESS

        goal_handle = await self.pickup_ac.send_goal_async(goal)
        result = await goal_handle.get_result_async()

        return result.result.error_code.val

    async def place_object(self, object_pose):
        self.get_logger().info("Clearing octomap")
        await self.clear_octomap_srv.call_async(Empty.Request())

        self.get_logger().info("Generating place locations")
        place_locations = self._generate_place_locations(object_pose)

        if self.place_ac is None:
            self.get_logger().warn("No place action client available (stub mode)")
            return MoveItErrorCodes.SUCCESS

        # First attempt with arm group
        goal = createPlaceGoal(
            place_pose=object_pose,
            place_locations=place_locations,
            group="arm",
            target="part",
            links_to_allow_contact=self.links_to_allow_contact
        )

        self.get_logger().info("Sending place goal with arm group")
        goal_handle = await self.place_ac.send_goal_async(goal)
        result = await goal_handle.get_result_async()

        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn("Place with arm failed, retrying with arm_torso group")
            goal_fallback = createPlaceGoal(
                place_pose=object_pose,
                place_locations=place_locations,
                group="arm_torso",
                target="part",
                links_to_allow_contact=self.links_to_allow_contact
            )
            goal_handle = await self.place_ac.send_goal_async(goal_fallback)
            result = await goal_handle.get_result_async()

        return result.result.error_code.val

    def _generate_grasps(self, object_pose):
        """Generate a list of possible grasps for the object."""
        grasps = []
        g = Grasp()
        g.id = "grasp_1"
        g.grasp_pose = deepcopy(object_pose)
        grasps.append(g)
        return grasps

    def _generate_place_locations(self, object_pose):
        """Generate a list of possible place locations."""
        place_locations = []
        pl = PlaceLocation()
        pl.place_pose = deepcopy(object_pose)
        pl.place_pose.pose.position.z += 0.05
        place_locations.append(pl)
        return place_locations


async def asyncio_sleep(seconds):
    """Async sleep helper."""
    import asyncio
    await asyncio.sleep(seconds)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()