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
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import SyncParametersClient
from rclpy.parameter import Parameter
from sys import exit
import time


class CameraReconfigure(object):
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('camera_reconfigure_client')
        # Assuming the target camera node is named 'camera'
        self.client = SyncParametersClient(self.node, 'camera')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Camera node not available')
            exit(-1)

    def disable_auto(self):
        # Disable automatic exposure and white balance
        parameters = [
            Parameter('exposure_auto', Parameter.Type.BOOL, False),
            Parameter('white_balance_auto', Parameter.Type.BOOL, False)
        ]
        self.client.set_parameters(parameters)

    def enable_auto(self):
        # Enable automatic exposure and white balance
        parameters = [
            Parameter('exposure_auto', Parameter.Type.BOOL, True),
            Parameter('white_balance_auto', Parameter.Type.BOOL, True)
        ]
        self.client.set_parameters(parameters)

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        exit(-1)

    reconfigure = CameraReconfigure()

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    time.sleep(1)