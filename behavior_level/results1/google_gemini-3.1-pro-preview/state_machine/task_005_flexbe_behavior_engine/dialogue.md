# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
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
Navigates a robot to a desired position and orientation using move_base.

Created on 11/19/2015

@author: Spyros Maniatopoulos
"""
from flexbe_core import EventState, Logger
# from flexbe_core.proxy import ProxyActionClient

from actionlib_msgs.msg import GoalStatus
# from nav2_msgs.msg import MoveBaseGoal    - Deprecated in ROS 2
# from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
# from tf_conversions import transformations


class MoveBaseState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    ># waypoint     Pose2D        Target waypoint for navigation.

    <= arrived                    Navigation to target pose succeeded.
    <= failed                     Navigation to target pose failed.
    """

    def __init__(self):
        """Constructor"""
        # TODO 1: [ROS 2 NAV2 INITIALIZATION]
    # 1. Initialize self._client as a ProxyActionClient with the following mapping:
    #    {'/navigate_to_pose': NavigateToPose}. 
    # 2. Use the exact topic string '/navigate_to_pose' with a leading slash.
    # [STYLE]: DO NOT use type hints. Keep the class name as 'MoveBaseState'.
        #END OF TODO
    def execute(self, userdata):
        # TODO 2: [ROS 2 FLEXBE EXECUTION]
    # 1. Implement non-blocking polling using self._client.has_result(self._action_topic).
    # 2. DO NOT use 'get_result_async()' or manually manage Action futures.
    # 3. Retrieve the status using self._client.get_state(self._action_topic).
    # 4. Map 'GoalStatus.STATUS_SUCCEEDED' to 'arrived' and failures to 'failed'.
        # END OF TODO
    def on_enter(self, userdata):
        """Create and send action goal"""

        self._failed = True
        Logger.error("move_base_state is deprecated! - MoveBase is not available in ROS 2")
        self._arrived = False
        self._failed = False

        raise NotImplementedError("MoveBase is not available in ROS 2 at this time!")

        # Create and populate action goal

        # Old ROS 1 code
        # goal = MoveBase_Goal()
        # pt = Point(x = userdata.waypoint.x, y = userdata.waypoint.y)
        # qt = transformations.quaternion_from_euler(0, 0, userdata.waypoint.theta)

        # goal.target_pose.pose = Pose(position = pt,
        #                              orientation = Quaternion(*qt))

        # goal.target_pose.header.frame_id = "odom"
        # goal.target_pose.header.stamp.secs = 5.0

        # # Send the action goal for execution
        # try:
        #     self._client.send_goal(self._action_topic, goal)
        # except Exception as e:
        #     Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
        #     self._failed = True

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

----------------------------


# LLM Response

```python
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
Navigates a robot to a desired position and orientation using move_base.

Created on 11/19/2015

@author: Spyros Maniatopoulos
"""
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
# from tf_conversions import transformations


class MoveBaseState(EventState):
    """
    Navigates a robot to a desired position and orientation using move_base.

    ># waypoint     Pose2D        Target waypoint for navigation.

    <= arrived                    Navigation to target pose succeeded.
    <= failed                     Navigation to target pose failed.
    """

    def __init__(self):
        """Constructor"""
        super(MoveBaseState, self).__init__(outcomes=['arrived', 'failed'],
                                            input_keys=['waypoint'])
        self._action_topic = '/navigate_to_pose'
        self._client = ProxyActionClient({self._action_topic: NavigateToPose})
        self._failed = False
        self._arrived = False

    def execute(self, userdata):
        if self._failed:
            return 'failed'

        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.STATUS_SUCCEEDED:
                return 'arrived'
            else:
                return 'failed'

    def on_enter(self, userdata):
        """Create and send action goal"""

        self._failed = True
        Logger.error("move_base_state is deprecated! - MoveBase is not available in ROS 2")
        self._arrived = False
        self._failed = False

        raise NotImplementedError("MoveBase is not available in ROS 2 at this time!")

        # Create and populate action goal

        # Old ROS 1 code
        # goal = MoveBase_Goal()
        # pt = Point(x = userdata.waypoint.x, y = userdata.waypoint.y)
        # qt = transformations.quaternion_from_euler(0, 0, userdata.waypoint.theta)

        # goal.target_pose.pose = Pose(position = pt,
        #                              orientation = Quaternion(*qt))

        # goal.target_pose.header.frame_id = "odom"
        # goal.target_pose.header.stamp.secs = 5.0

        # # Send the action goal for execution
        # try:
        #     self._client.send_goal(self._action_topic, goal)
        # except Exception as e:
        #     Logger.logwarn("Unable to send navigation action goal:\n%s" % str(e))
        #     self._failed = True

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
```