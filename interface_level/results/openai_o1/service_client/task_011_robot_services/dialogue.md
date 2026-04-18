# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python

# Copyright (C) 2015 Fetch Robotics Inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Michael Ferguson

import sys
import rospy
import dynamic_reconfigure.client
from sys import exit


class CameraReconfigure(object):
# TODO:define methods of this class that
# Initialize a ROS2-side interface
# Disable automatic exposure and white balance
# Enable automatic exposure and white balance
# END OF TODO


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        exit(-1)

    rospy.init_node("camera_reconfigure")
    reconfigure = CameraReconfigure()

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    rospy.sleep(1)

----------------------------


# LLM Response

#!/usr/bin/env python3

import sys
import rclpy
import time
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType


class CameraReconfigure(object):
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('camera_reconfigure')
        self.client = self.node.create_client(SetParameters, '/camera_driver/set_parameters')
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass

    def enable_auto(self):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name='exposure_auto',
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
            ),
            Parameter(
                name='white_balance_auto',
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
            ),
        ]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

    def disable_auto(self):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name='exposure_auto',
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
            ),
            Parameter(
                name='white_balance_auto',
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
            ),
        ]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        sys.exit(-1)

    reconfigure = CameraReconfigure()

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    time.sleep(1)
    reconfigure.node.destroy_node()
    rclpy.shutdown()