# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from your_package.srv import AddThreeInts

node = None

def handle_add_three_ints(request, response):
    global node
    response.sum = request.a + request.b + request.c
    node.get_logger().info(f"Received request: {request.a} {request.b} {request.c}")
    return response

def server_node():
    global node
    rclpy.init()
    node = Node("add_three_ints_server")
    node.create_service(AddThreeInts, 'add_three_ints', handle_add_three_ints)
    node.get_logger().info("Custom service server started.")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    server_node()