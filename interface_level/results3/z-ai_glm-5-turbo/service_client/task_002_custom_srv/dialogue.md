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
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from your_package.srv import AddThreeInts

def client_node():
    rospy.init_node("add_three_ints_client")
    # TODO: wait for service
    # call service
    # END OF TODO
    rospy.loginfo("Client executed.")

if __name__ == "__main__":
    client_node()

----------------------------

FILE_PATH: ros_server.py
----------------------------
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from your_package.srv import AddThreeInts, AddThreeIntsResponse

def handle_add_three_ints(req):
    # TODO: fill computation
    rospy.loginfo("Received request: %s %s %s", req.a, req.b, req.c)
    # return correct response
    return  
    # END OF TODO

def server_node():
    rospy.init_node("add_three_ints_server")
    # TODO: advertise service
    rospy.loginfo("Custom service server started.")
    rospy.spin()
    # END OF TODO
if __name__ == "__main__":
    server_node()

----------------------------


# LLM Response

[FILENAME: ros_client.py]
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from your_package.srv import AddThreeInts

def client_node():
    rclpy.init()
    node = Node("add_three_ints_client")
    client = node.create_client(AddThreeInts, 'add_three_ints')
    # TODO: wait for service
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    # call service
    req = AddThreeInts.Request()
    req.a = 1
    req.b = 2
    req.c = 3
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    # END OF TODO
    node.get_logger().info("Client executed.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    client_node()

[FILENAME: ros_server.py]
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from your_package.srv import AddThreeInts, AddThreeIntsResponse

def handle_add_three_ints(req, node):
    # TODO: fill computation
    node.get_logger().info("Received request: %s %s %s", req.a, req.b, req.c)
    # return correct response
    return AddThreeInts.Response(sum=req.a + req.b + req.c)
    # END OF TODO

def server_node():
    rclpy.init()
    node = Node("add_three_ints_server")
    # TODO: advertise service
    srv = node.create_service(AddThreeInts, 'add_three_ints', lambda req, res: handle_add_three_ints(req, node))
    node.get_logger().info("Custom service server started.")
    rclpy.spin(node)
    # END OF TODO
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    server_node()