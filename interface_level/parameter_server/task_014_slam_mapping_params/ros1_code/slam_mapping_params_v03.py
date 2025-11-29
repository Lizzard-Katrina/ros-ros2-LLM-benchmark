#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node("slam_mapping_params_v03")

    # TODO: restore all critical SLAM parameters
    map_update_rate = rospy.get_param("~map_update_rate", )
    max_range = rospy.get_param("~max_range", )
    odom_frame = rospy.get_param("~odom_frame", )
    # END

    rospy.loginfo(f"[v03] Map Update Rate: {map_update_rate}, Max Range: {max_range}, Odom Frame: {odom_frame}")

if __name__ == "__main__":
    main()
