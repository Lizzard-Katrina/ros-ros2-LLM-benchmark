#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node("px4_params_v02")

    # TODO: restore missing parameters and dependent computation
    noise_sigma = rospy.get_param("~noise_sigma", )
    control_gain = rospy.get_param("~control_gain",)
    max_vel = rospy.get_param("~max_vel",)
    control_output =
    # END

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rospy.loginfo(f"[v02] control_output={control_output}")
        rate.sleep()

if __name__ == "__main__":
    main()
