class UnlockedState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['lock'])
        self.push_attempt = False
        self.pub = rospy.Publisher('state_info', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo("Unlocked state")
        while not self.push_attempt:
            # --------------------------
            # TODO: Translate this ROS1 push check and publishing to ROS2
            # END
            rospy.sleep(0.5)
        return 'lock'
