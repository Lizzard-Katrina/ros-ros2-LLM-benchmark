#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node("px4_params_v01")

    # TODO: restore missing parameter
    rate_hz = rospy.get_param("~rate_hz",)
    # END

    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        rospy.loginfo(f"[v01] Rate = {rate_hz}")
        rate.sleep()

if __name__ == "__main__":
    main()
