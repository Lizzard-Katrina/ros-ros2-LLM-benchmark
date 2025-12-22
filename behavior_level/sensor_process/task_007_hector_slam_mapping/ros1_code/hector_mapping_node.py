#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

def map_callback(msg):
    rospy.loginfo("Map updated. Size: {}x{}".format(msg.info.width, msg.info.height))

def pose_callback(msg):
    rospy.loginfo("Pose received: x={}, y={}".format(
        msg.pose.position.x,
        msg.pose.position.y
    ))

def main():
    rospy.init_node("hector_mapping_monitor")

    # TODO: subscribe to /map and /pose topics
    # END: topic subscriptions end here

    rospy.spin()

if __name__ == "__main__":
    main()
