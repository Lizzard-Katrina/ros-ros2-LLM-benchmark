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
import time
from sys import exit

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParametersClient


class CameraReconfigure(object):
    def __init__(self):
        self.node = Node("camera_reconfigure")
        target_node = self.node.declare_parameter(
            "target_node", "/head_camera/driver"
        ).get_parameter_value().string_value
        self.client = AsyncParametersClient(self.node, target_node)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for parameter service...")

    def disable_auto(self):
        params = [
            Parameter("auto_exposure", value=False),
            Parameter("auto_white_balance", value=False),
        ]
        future = self.client.set_parameters(params)
        rclpy.spin_until_future_complete(self.node, future)

    def enable_auto(self):
        params = [
            Parameter("auto_exposure", value=True),
            Parameter("auto_white_balance", value=True),
        ]
        future = self.client.set_parameters(params)
        rclpy.spin_until_future_complete(self.node, future)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        exit(-1)

    rclpy.init()
    reconfigure = CameraReconfigure()

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    time.sleep(1.0)

    reconfigure.node.destroy_node()
    rclpy.shutdown()