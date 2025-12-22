#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

class ObstacleDetector:
    def __init__(self):
        rospy.loginfo("Starting obstacle detector node")

        self.pub = rospy.Publisher("/obstacles", PointStamped, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        min_range = min(msg.ranges)

        # TODO: fill in how to build obstacle position message
        obstacle_msg = <BLANK>
        # END: fill here

        self.pub.publish(obstacle_msg)

if __name__ == "__main__":
    rospy.init_node("laser_obstacle_detector")
    node = ObstacleDetector()
    rospy.spin()
