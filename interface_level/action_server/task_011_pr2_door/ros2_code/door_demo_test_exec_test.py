Here is the ROS2 version of the provided ROS1 code:

```python
#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

## Gazebo collision validation 
PKG = 'pr2_doors_gazebo_demo'
NAME = 'test_door_no_executive'

import unittest
import sys
import os
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from move_base_msgs.action import MoveBase
from door_msgs.action import Door

TEST_DURATION = 60000.0


class TestDoorNoExecutive(Node):
    def __init__(self):
        super().__init__('open_door_test')

        # initial door
        prior_door = Door.Door()
        prior_door.frame_p1.x = 1.0
        prior_door.frame_p1.y = -0.5
        prior_door.frame_p2.x = 1.0
        prior_door.frame_p2.y = 0.5
        prior_door.door_p1.x = 1.0
        prior_door.door_p1.y = -0.5
        prior_door.door_p2.x = 1.0
        prior_door.door_p2.y = 0.5
        prior_door.travel_dir.x = 1.0
        prior_door.travel_dir.y = 0.0
        prior_door.travel_dir.z = 0.0
        prior_door.rot_dir = Door.Door.ROT_DIR_COUNTERCLOCKWISE
        prior_door.hinge = Door.Door.HINGE_P2
        prior_door.header.frame_id = "base_footprint"
        self.door = Door.DoorGoal()
        self.door.door = prior_door

        self.move = MoveBase.Goal()
        self.move.target_pose.header.frame_id = 'odom_combined'
        self.move.target_pose.pose.position.x = 10
        self.move.target_pose.pose.position.y = 10
        self.move.target_pose.pose.orientation.w = 1

        self.ac_door = ActionClient(self, Door, 'door')
        self.ac_move = ActionClient(self, MoveBase, 'move_base')

        self.sub = self.create_subscription(String, "/test_output", self.stringOutput, qos_profile_sensor_data)

    def stringOutput(self, str):
        self.get_logger().info(str.data)

    def test_door_no_executive(self):
        self.ac_door.wait_for_server()
        self.ac_move.wait_for_server()
        self.assert_(self.ac_door.send_goal_and_wait(self.door, TEST_DURATION, 5.0))
        self.assert_(self.ac_move.send_goal_and_wait(self.move, TEST_DURATION, 5.0))


def main(args=None):
    rclpy.init(args=args)
    test = TestDoorNoExecutive()
    try:
        test.test_door_no_executive()
    except KeyboardInterrupt:
        test.get_logger().info('KeyboardInterrupt caught')
    except ExternalShutdownException:
        test.get_logger().info('ExternalShutdownException caught')
    finally:
        test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()