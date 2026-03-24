#!/usr/bin/env python3

# Copyright 2023 Philipp Schillinger,  Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger,  Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
Navigates a robot to a desired position and orientation using move_base.

Created on 11/19/2015

@author: Spyros Maniatopoulos
"""
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf_transformations


class MoveBaseState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    ># waypoint     Pose2D        Target waypoint for navigation.

    <= arrived                    Navigation to target pose succeeded.
    <= failed                     Navigation to target pose failed.
    """

    def __init__(self):
        """Constructor"""

        self._action_topic = 'navigate_to_pose'
        self._client = ProxyActionClient({self._action_topic: NavigateToPose})
        self._goal_handle = None
        self._result_future = None
        self._arrived = False
        self._failed = False

    def execute(self, userdata):
        if self._result_future is None:
            return None

        if self._result_future.done():
            result = self._result_future.result()
            status = self._goal_handle.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self._arrived = True
                return 'arrived'
            else:
                self._failed = True
                return 'failed'
        return None

    def on_enter(self, userdata):
        """Create and send action goal"""

        self._failed = False
        self._arrived = False

        goal_msg = NavigateToPose.Goal()
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self._node.get_clock().now().to_msg()

        x = userdata.waypoint.x
        y = userdata.waypoint.y
        theta = userdata.waypoint.theta

        quat = tf_transformations.quaternion_from_euler(0, 0, theta)

        pose_stamped.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )

        goal_msg.pose = pose_stamped

        try:
            self._goal_handle = self._client.send_goal(self._action_topic, goal_msg)
            self._result_future = self._goal_handle.get_result_async()
        except Exception as e:
            Logger.logwarn(f"Unable to send navigation action goal:\n{str(e)}")
            self._failed = True

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('Cancelled move_base active action goal.')

    def on_exit(self, userdata):
        self.cancel_active_goals()

    def on_stop(self):
        self.cancel_active_goals()