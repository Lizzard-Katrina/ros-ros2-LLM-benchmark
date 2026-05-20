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

FILE_PATH: camera_reconfigure.py
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

[FILENAME: camera_reconfigure.py]
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
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters


class CameraReconfigure(object):
    def __init__(self):
        self.node = Node("camera_reconfigure")
        self.camera_node = self.node.declare_parameter("camera_node", "/head_camera").value
        self.client = self.node.create_client(SetParameters, self.camera_node + "/set_parameters")

        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn(
                "Parameter service '%s/set_parameters' is not available" % self.camera_node
            )

    def _set_parameters(self, values):
        request = SetParameters.Request()

        for name, value in values.items():
            parameter = Parameter()
            parameter.name = name
            parameter.value = ParameterValue()

            if isinstance(value, bool):
                parameter.value.type = ParameterType.PARAMETER_BOOL
                parameter.value.bool_value = value
            elif isinstance(value, int):
                parameter.value.type = ParameterType.PARAMETER_INTEGER
                parameter.value.integer_value = value
            elif isinstance(value, float):
                parameter.value.type = ParameterType.PARAMETER_DOUBLE
                parameter.value.double_value = value
            elif isinstance(value, str):
                parameter.value.type = ParameterType.PARAMETER_STRING
                parameter.value.string_value = value
            else:
                self.node.get_logger().warn("Unsupported parameter type for '%s'" % name)
                continue

            request.parameters.append(parameter)

        if not request.parameters:
            return

        if not self.client.service_is_ready():
            if not self.client.wait_for_service(timeout_sec=5.0):
                self.node.get_logger().error(
                    "Unable to contact parameter service '%s/set_parameters'" % self.camera_node
                )
                return

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.result() is None:
            self.node.get_logger().error("Failed to set camera parameters")
            return

        for index, result in enumerate(future.result().results):
            if not result.successful:
                self.node.get_logger().warn(
                    "Failed to set parameter '%s': %s"
                    % (request.parameters[index].name, result.reason)
                )

    def disable_auto(self):
        self._set_parameters(
            {
                "auto_exposure": False,
                "auto_white_balance": False,
                "exposure_auto": False,
                "white_balance_auto": False,
                "enable_auto_exposure": False,
                "enable_auto_white_balance": False,
            }
        )

    def enable_auto(self):
        self._set_parameters(
            {
                "auto_exposure": True,
                "auto_white_balance": True,
                "exposure_auto": True,
                "white_balance_auto": True,
                "enable_auto_exposure": True,
                "enable_auto_white_balance": True,
            }
        )


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

    rclpy.spin_once(reconfigure.node, timeout_sec=1.0)
    reconfigure.node.destroy_node()
    rclpy.shutdown()