#!/usr/bin/env python3
import rospy
# TODO: import correct service types
from controller_manager_msgs.srv import ListControllers, SwitchController

def controller_manager_client():
    # Wait for controller_manager services
    rospy.wait_for_service("/controller_manager/list_controllers")
    rospy.wait_for_service("/controller_manager/switch_controller")
    
    try:
        # TODO: create service proxies
        list_srv = ________  # ListControllers service proxy
        switch_srv = ________  # SwitchController service proxy

        # TODO: call list controllers
        resp_list = ________
        rospy.loginfo("Controllers available: %s", resp_list.controllers)

        # TODO: call switch_controller to start/stop controllers
        resp_switch = ________
        rospy.loginfo("Switch controller response: %s", resp_switch)

    except Exception as e:
        rospy.logerr("Controller manager service call failed: ________")

if __name__ == "__main__":
    rospy.init_node("controller_manager_client_node")
    controller_manager_client()
