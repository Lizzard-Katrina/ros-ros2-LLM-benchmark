#!/usr/bin/env python3
import rospy
# TODO: import correct service types
# Example: from fetch_api.srv import GripperCommand, MoveHead
# Example: from pr2_common.srv import HeadTrajectory, GripperCommand

def robot_service_client():
    # Wait for robot-specific services
    rospy.wait_for_service("/gripper_command")
    rospy.wait_for_service("/move_head")
    rospy.wait_for_service("/arm_command")  # if applicable

    try:
        # TODO: create service proxies for multiple services
        gripper_srv = ________  # GripperCommand proxy
        head_srv = ________     # MoveHead proxy
        arm_srv = ________      # ArmCommand proxy

        # TODO: construct requests for each service
        gripper_req = ________
        head_req = ________
        arm_req = ________

        # TODO: call services sequentially
        gripper_resp = ________
        rospy.loginfo("Gripper response: %s", gripper_resp)

        head_resp = ________
        rospy.loginfo("Head response: %s", head_resp)

        arm_resp = ________
        rospy.loginfo("Arm response: %s", arm_resp)

    except Exception as e:
        rospy.logerr("Robot service call failed: ________")

if __name__ == "__main__":
    rospy.init_node("robot_service_client_node")
    robot_service_client()
