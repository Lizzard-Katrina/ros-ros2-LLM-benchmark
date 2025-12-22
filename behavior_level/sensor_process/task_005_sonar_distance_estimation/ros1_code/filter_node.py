#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

class DistanceFilter:
    def __init__(self):
        # TODO: initialize subscriber to /sonar/raw AND publisher to /distance_filtered
        # END: filter initialization and publisher setup

    def process(self, msg):
        # Placeholder smoothing logic
        rospy.loginfo("Filtering sonar data...")

if __name__ == "__main__":
    rospy.init_node("sonar_distance_filter")
    node = DistanceFilter()
    rospy.spin()
