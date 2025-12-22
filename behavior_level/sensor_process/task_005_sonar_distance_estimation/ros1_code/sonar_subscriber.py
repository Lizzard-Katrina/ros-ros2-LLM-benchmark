#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

class SonarSubscriber:
    def __init__(self):
        # TODO: subscribe to /sonar/raw
        # END: sonar subscriber initialization

    def callback(self, msg):
        rospy.loginfo("Sonar raw distance: %f" % msg.data)

if __name__ == "__main__":
    rospy.init_node("sonar_subscriber_node")
    node = SonarSubscriber()
    rospy.spin()
