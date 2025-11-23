#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# TODO: import service proxy AddThreeInts

def client_node():
    rospy.init_node("add_three_ints_client")
    # TODO: wait for service
    # TODO: call service
    rospy.loginfo("Client executed.")

if __name__ == "__main__":
    client_node()
