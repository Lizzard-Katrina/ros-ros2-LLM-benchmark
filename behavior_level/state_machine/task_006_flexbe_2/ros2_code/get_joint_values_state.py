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
Retrieves current values of specified joints.
Created on 06.03.2016

@author: Philipp Schillinger
"""

from rclpy.duration import Duration

from flexbe_core import EventState
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import JointState


class GetJointValuesState(EventState):
    """
    Retrieves current values of specified joints.

    -- joints                string[]    List of desired joint names.
    -- timeout                double        Timeout value (optional)
    -- joint_states_topic    string        Optional name of joint states topic
                                        (default: /joint_states)

    #> joint_values float[]     List of current joint values.

    <= retrieved                 Joint values are available.

    """

    def __init__(self, joints, timeout=None, joint_states_topic='/joint_states'):
        """
        Constructor
        """
        super().__init__(outcomes=['retrieved', 'timeout'],
                         output_keys=['joint_values'])

        self._topic = joint_states_topic
        self._sub = ProxySubscriberCached({self._topic: JointState}, inst_id=id(self))

        self._joints = joints
        self._joint_values = []
        self._return_code = None
        self._timeout = Duration(seconds=timeout) if timeout is not None else None
        self._start_time = None

    def execute(self, userdata):

        while self._sub.has_buffered(self._topic):
            msg = self._sub.get_from_buffer(self._topic)
            name_to_pos = dict(zip(msg.name, msg.position))
            found_all = True
            joint_values = []
            for joint in self._joints:
                if joint in name_to_pos:
                    joint_values.append(name_to_pos[joint])
                else:
                    found_all = False
                    break
            if found_all:
                self._joint_values = joint_values
                userdata.joint_values = self._joint_values
                return 'retrieved'

        if self._timeout is not None and self._start_time is not None:
            elapsed = self.get_clock().now() - self._start_time
            if elapsed > self._timeout:
                return 'timeout'

    def on_enter(self, userdata):
        self._sub.enable_buffer(self._topic)
        self._joint_values = [None] * len(self._joints)
        self._start_time = self.get_clock().now()

    def on_exit(self, userdata):
        self._sub.disable_buffer(self._topic)