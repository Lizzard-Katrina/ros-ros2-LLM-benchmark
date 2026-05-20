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
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration


class CameraReconfigure(object):
    def __init__(self):
        self.node = Node('camera_reconfigure')
        self.client = self.node.create_client(
            SetParameters, 
            '/camera/driver/set_parameters'
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

    def _set_parameters(self, params_dict):
        request = SetParameters.Request()
        request.parameters = []
        
        for name, value in params_dict.items():
            param = Parameter()
            param.name = name
            if isinstance(value, bool):
                param.value.type = Parameter.Type.BOOL
                param.value.bool_value = value
            elif isinstance(value, int):
                param.value.type = Parameter.Type.INTEGER
                param.value.integer_value = value
            elif isinstance(value, float):
                param.value.type = Parameter.Type.DOUBLE
                param.value.double_value = value
            elif isinstance(value, str):
                param.value.type = Parameter.Type.STRING
                param.value.string_value = value
            
            request.parameters.append(param)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def disable_auto(self):
        self.node.get_logger().info("Disabling automatic exposure and white balance")
        params = {
            'auto_exposure': False,
            'auto_white_balance': False
        }
        self._set_parameters(params)

    def enable_auto(self):
        self.node.get_logger().info("Enabling automatic exposure and white balance")
        params = {
            'auto_exposure': True,
            'auto_white_balance': True
        }
        self._set_parameters(params)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: camera_reconfigure --enable/disable")
        sys.exit(-1)

    rclpy.init()
    reconfigure = CameraReconfigure()

    if sys.argv[1] == "--enable":
        reconfigure.enable_auto()
    else:
        reconfigure.disable_auto()

    # Sleep for 1 second to allow service call to complete
    time.sleep(1)
    
    reconfigure.node.destroy_node()
    rclpy.shutdown()