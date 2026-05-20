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

FILE_PATH: mp3_controller.py
----------------------------
#!/usr/bin/env python

#  Software License Agreement (BSD License)
#  
#  Copyright (c) 2015, Jan Winkler, Institute for Artificial Intelligence,
#  Universitaet Bremen.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Institute for Artificial Intelligence,
#     Universitaet Bremen, nor the names of its contributors may be
#     used to endorse or promote products derived from this software
#     without specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('ros_service_examples')

import sys

from ros_service_examples.srv import *
import rospy


def mp3_inventory_client():
  rospy.wait_for_service('mp3_inventory_interaction')
  

  #TODO:
  #- Initialize a ROS2 service client for the MP3 inventory service
  #- Query the service for inventory information using the provided interface
  #- Use the service responses to determine subsequent requests and control flow
  #- Log or print the inventory information obtained from the service

  try:
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  #END OF TODO

if __name__ == "__main__":
  mp3_inventory_client()

----------------------------


# LLM Response

[FILENAME: mp3_controller.py]
#!/usr/bin/env python3

#  Software License Agreement (BSD License)
#  
#  Copyright (c) 2015, Jan Winkler, Institute for Artificial Intelligence,
#  Universitaet Bremen.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Institute for Artificial Intelligence,
#     Universitaet Bremen, nor the names of its contributors may be
#     used to endorse or promote products derived from this software
#     without specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import sys
import rclpy
from rclpy.node import Node
from ros_service_examples.srv import Mp3InventoryInteraction


def mp3_inventory_client():
    rclpy.init()
    node = Node('mp3_inventory_client')
    
    client = node.create_client(Mp3InventoryInteraction, 'mp3_inventory_interaction')
    
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Service not available, waiting again...')
        rclpy.shutdown()
        return

    #TODO:
    #- Initialize a ROS2 service client for the MP3 inventory service
    #- Query the service for inventory information using the provided interface
    #- Use the service responses to determine subsequent requests and control flow
    #- Log or print the inventory information obtained from the service

    try:
        request = Mp3InventoryInteraction.Request()
        request.query = "list_all"
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() is not None:
            response = future.result()
            node.get_logger().info("Inventory Information Received:")
            node.get_logger().info(f"Status: {response.status}")
            node.get_logger().info(f"Tracks: {response.tracks}")
            node.get_logger().info(f"Total Duration: {response.total_duration}")
            
            if response.tracks:
                node.get_logger().info("Available tracks for playback control:")
                for track in response.tracks:
                    node.get_logger().info(f" - {track}")
        else:
            node.get_logger().error("Service call failed to return a valid response")
            
    except Exception as e:
        node.get_logger().error(f"Service call failed: {str(e)}")
    #END OF TODO
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    mp3_inventory_client()