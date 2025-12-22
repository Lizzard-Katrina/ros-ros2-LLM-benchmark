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
        min_index = msg.ranges.index(min_range)

        obstacle_msg = PointStamped()
        obstacle_msg.header = msg.header
        obstacle_msg.point.x = min_range
        obstacle_msg.point.y = 0.0
        obstacle_msg.point.z = 0.0

        self.pub.publish(obstacle_msg)

if __name__ == "__main__":
    rospy.init_node("laser_obstacle_detector")
    node = ObstacleDetector()
    rospy.spin()
