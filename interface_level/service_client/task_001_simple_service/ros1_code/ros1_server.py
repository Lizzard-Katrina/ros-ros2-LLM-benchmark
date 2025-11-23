#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    # TODO: AI/user completes service logic
    rospy.loginfo("Server received request: %s + %s", req.a, req.b)
    return AddTwoIntsResponse()  # leave empty

def server_node():
    rospy.init_node('add_two_ints_server')
    # TODO: advertise the service
    rospy.spin()

if __name__ == "__main__":
    server_node()
