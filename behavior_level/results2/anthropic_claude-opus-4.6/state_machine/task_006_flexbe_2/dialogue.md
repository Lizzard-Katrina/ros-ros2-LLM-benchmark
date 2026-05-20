# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: get_joint_values_state.py
----------------------------
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
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
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
        self._timeout = Duration(seconds=timeout)

    def execute(self, userdata):

        # TODO 1: Process the buffered joint state messages. 
        # 1. MUST use 'self._sub.has_buffered()' and 'self._sub.get_from_buffer()'.
        # 2. Map 'msg.position' values to 'self._joint_values' using 'msg.name'.
        # 3. Only return 'retrieved' when ALL target joints have been found.
        # [STYLE]: DO NOT use 'get_last_msg'. You MAY use 'zip()', 'dict()', or 'index()'. 
        # MANDATORY: Use 'self.get_clock().now()' for any time calculations
        #END OF TODO

    def on_enter(self, userdata):
        # TODO 2: Prepare the state for a new execution cycle.
        # 1. Enable the proxy buffer for the joint states topic.
        # 2. Initialize 'self._joint_values' as a list of 'None' matching 
        #    the length of 'self._joints'.
        # 3. Set the start time using the node's clock.
        # [STYLE]: DO NOT instantiate 'Clock()' manually. Use 'self.get_clock()'.
        # END OF TODO
    def on_exit(self, userdata):
        self._sub.disable_buffer(self._topic)

----------------------------


# LLM Response

[FILENAME: get_joint_values_state.py]
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
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
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

        if self._timeout is not None:
            elapsed = self.get_clock().now() - self._start_time
            if elapsed > self._timeout:
                userdata.joint_values = self._joint_values
                return 'timeout'

        while self._sub.has_buffered(self._topic):
            msg = self._sub.get_from_buffer(self._topic)
            name_to_position = dict(zip(msg.name, msg.position))
            for i, joint_name in enumerate(self._joints):
                if joint_name in name_to_position:
                    self._joint_values[i] = name_to_position[joint_name]

        if all(v is not None for v in self._joint_values):
            userdata.joint_values = list(self._joint_values)
            return 'retrieved'

    def on_enter(self, userdata):
        self._sub.enable_buffer(self._topic)
        self._joint_values = [None] * len(self._joints)
        self._start_time = self.get_clock().now()

    def on_exit(self, userdata):
        self._sub.disable_buffer(self._topic)