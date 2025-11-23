#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# TODO: import service type AddThreeInts

def handle_add_three_ints(req):
    # TODO: fill computation
    rospy.loginfo("Received request: %s %s %s", req.a, req.b, req.c)
    return  # TODO: return correct response

def server_node():
    rospy.init_node("add_three_ints_server")
    # TODO: advertise service
    rospy.loginfo("Custom service server started.")
    rospy.spin()

if __name__ == "__main__":
    server_node()
