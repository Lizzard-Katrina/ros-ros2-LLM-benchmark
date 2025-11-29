#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node("slam_mapping_params_v01")

    # TODO: restore missing parameter
    map_update_rate = rospy.get_param("~map_update_rate", )
    # END

    rospy.loginfo(f"[v01] Map Update Rate: {map_update_rate}")

if __name__ == "__main__":
    main()
