#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from beginner_tutorials.srv import AddTwoInts

def client_node():
    rospy.init_node('add_two_ints_client')
    # TODO: wait for service and call
    rospy.loginfo("Client node running")

if __name__ == "__main__":
    client_node()
