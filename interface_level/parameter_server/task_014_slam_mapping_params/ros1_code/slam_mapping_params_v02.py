#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node("slam_mapping_params_v02")

    # TODO: restore multiple parameters
    map_update_rate = rospy.get_param("~map_update_rate", )
    max_range = rospy.get_param("~max_range", )
    # END

    rospy.loginfo(f"[v02] Map Update Rate: {map_update_rate}, Max Range: {max_range}")

if __name__ == "__main__":
    main()
