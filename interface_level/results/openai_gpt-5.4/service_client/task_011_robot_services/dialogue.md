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
from sys import exit

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


class CameraReconfigure(object):
    def __init__(self):
        self.node = Node("camera_reconfigure_client")
        self.client = self.node.create_client(SetParameters, "/camera/set_parameters")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /camera/set_parameters service...")

    def _set_parameters(self, params_dict):
        request = SetParameters.Request()
        request.parameters = []

        for name, value in params_dict.items():
            param = Parameter()
            param.name = name
            param.value = ParameterValue()

            if isinstance(value, bool):
                param.value.type = ParameterType.PARAMETER_BOOL
                param.value.bool_value = value
            elif isinstance(value, int):
                param.value.type = ParameterType.PARAMETER_INTEGER
                param.value.integer_value = value
            elif isinstance(value, float):
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = value
            elif isinstance(value, str):
                param.value.type = ParameterType.PARAMETER_STRING
                param.value.string_value = value
            else:
                raise TypeError("Unsupported parameter type for {}: {}".format(name, type(value)))

            request.parameters.append(param)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def disable_auto(self):
        self._set_parameters({
            "auto_exposure": False,
            "auto_white_balance": False,
        })

    def enable_auto(self):
        self._set_parameters({
            "auto_exposure": True,
            "auto_white_balance": True,
        })


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

    reconfigure.node.create_timer(1.0, lambda: None)
    rclpy.spin_once(reconfigure.node, timeout_sec=1.0)

    reconfigure.node.destroy_node()
    rclpy.shutdown()