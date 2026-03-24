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
