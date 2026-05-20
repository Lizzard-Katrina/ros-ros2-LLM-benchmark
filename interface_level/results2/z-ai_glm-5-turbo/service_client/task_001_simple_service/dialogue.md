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

FILE_PATH: ros_client.py
----------------------------
import rospy 
from beginner_tutorials.srv 
import AddTwoInts 
def client_node(): 
    rospy.init_node('add_two_ints_client') 
    # TODO: wait for service and call 
    rospy.loginfo("Client node running") 
    #END OF TODO

if __name__ == "__main__": client_node()

----------------------------

FILE_PATH: ros_server.py
----------------------------
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    # TODO: AI/user completes service logic
    rospy.loginfo("Server received request: %s + %s", req.a, req.b)
    return AddTwoIntsResponse()
    # END OF TODO

def server_node():
    rospy.init_node('add_two_ints_server')
    # TODO: advertise the service
    rospy.spin()
    # END OF TODO


if __name__ == "__main__":
    server_node()

----------------------------


# LLM Response

[FILENAME: ros_client.py]
import rclpy 
from rclpy.node import Node 
from beginner_tutorials.srv import AddTwoInts 

def client_node(): 
    rclpy.init() 
    node = Node('add_two_ints_client') 
    client = node.create_client(AddTwoInts, 'add_two_ints') 
    while not client.wait_for_service(timeout_sec=1.0): 
        node.get_logger().info('Service not available, waiting again...') 
    req = AddTwoInts.Request() 
    req.a = 2 
    req.b = 3 
    future = client.call_async(req) 
    rclpy.spin_until_future_complete(node, future) 
    if future.result() is not None: 
        node.get_logger().info("Client node running") 
    else: 
        node.get_logger().error('Service call failed') 
    node.destroy_node() 
    rclpy.shutdown() 

if __name__ == "__main__": client_node()

[FILENAME: ros_server.py]
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req, node):
    node.get_logger().info("Server received request: %s + %s", req.a, req.b)
    return AddTwoIntsResponse(sum=req.a + req.b)

def server_node():
    rclpy.init()
    node = Node('add_two_ints_server')
    service = node.create_service(AddTwoInts, 'add_two_ints', lambda req, resp: handle_add_two_ints(req, node))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    server_node()