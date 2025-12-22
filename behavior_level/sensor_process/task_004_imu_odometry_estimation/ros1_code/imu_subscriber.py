#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

class IMUSubscriber:
    def __init__(self):
        # TODO: initialize subscriber to /imu/data_raw
        # END: imu subscriber initialization

    def callback(self, msg):
        rospy.loginfo("IMU data received")

if __name__ == "__main__":
    rospy.init_node("imu_subscriber_node")
    node = IMUSubscriber()
    rospy.spin()
