#!/usr/bin/env python
import rospy

class TurtleBot3ModelLoader:
    def __init__(self):
        rospy.init_node("turtlebot3_model_loader")

        # TODO: Load TurtleBot3 model parameters from parameter server
        # END_TODO

        rospy.loginfo("Loaded TurtleBot3 parameters: type=%s, wheel_diameter=%.3f, wheel_base=%.3f",
                      self.robot_type, self.wheel_diameter, self.wheel_base)

if __name__ == "__main__":
    loader = TurtleBot3ModelLoader()
