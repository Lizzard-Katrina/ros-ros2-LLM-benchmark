#!/usr/bin/env python3
import rospy
from task_002_custom_msg_basic.msg import Person

def callback(msg):
    # ======= STUDENT TODO ========
    # Print received data
    # =============================
    rospy.loginfo(f"Received: {msg}")

def main():
    rospy.init_node('person_subscriber')

    # ======= STUDENT TODO ========
    # Create a subscriber listening to /person_info
    # =============================
    sub = rospy.Subscriber('/person_info', Person, callback)

    rospy.spin()

if __name__ == '__main__':
    main()
