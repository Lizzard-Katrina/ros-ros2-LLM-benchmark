#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node("px4_params_v03")

    # TODO: restore parameter group + full computation block
    noise_sigma = rospy.get_param("~noise_sigma", )
    thrust_gain = rospy.get_param("~thrust_gain", )
    vel_limit = rospy.get_param("~vel_limit",)
    control_effort =
    # END

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo(f"[v03] control_effort={control_effort}")
        rate.sleep()

if __name__ == "__main__":
    main()
