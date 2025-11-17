#!/usr/bin/env python3
import rospy
from task_002_custom_msg_basic.msg import Person

def main():
    rospy.init_node('person_publisher')

    # ======= STUDENT TODO ========
    # Create a publisher named /person_info
    # publishing the custom Person message.
    # Fill the message fields and publish at 1 Hz.
    # =============================

    pub = rospy.Publisher('/person_info', Person, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg = Person()

        # ======= STUDENT TODO ========
        # Fill the message fields: name, age, height
        # =============================
        msg.name = "Alice"
        msg.age = 20
        msg.height = 1.65

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
