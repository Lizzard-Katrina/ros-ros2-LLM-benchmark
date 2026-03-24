#!/usr/bin/env python3
import rospy

# Mock of ROS1 custom message Person
class Person:
    def __init__(self):
        self.name = ""
        self.age = 0
        self.height = 0


def main():
    rospy.init_node('person_publisher')

    # ======= STUDENT TODO ========
    # Create a publisher named /person_info
    # publishing the custom Person message.
    # Fill the message fields and publish at 1 Hz.


    while not rospy.is_shutdown():
        msg = Person()

        # Fill the message fields: name, age, height
        # =============================
        rate.sleep()
        # END OF TODO
if __name__ == '__main__':
    main()
