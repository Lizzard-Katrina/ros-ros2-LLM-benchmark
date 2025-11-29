#!/usr/bin/env python
import rospy

class PR2CalibrationConfig:
    def __init__(self):
        rospy.init_node("pr2_calibration_config")

        # TODO: Load calibration parameters from parameter server
        # end:TODO

        rospy.loginfo("Calibration parameters loaded:")
        rospy.loginfo("  arm_offset: %s", self.arm_offset)
        rospy.loginfo("  gripper_zero: %s", self.gripper_zero)
        rospy.loginfo("  base_pitch_offset: %s", self.base_pitch_offset)


if __name__ == "__main__":
    config_node = PR2CalibrationConfig()
    rospy.spin()
