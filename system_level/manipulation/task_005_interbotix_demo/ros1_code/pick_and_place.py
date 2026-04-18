import time
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python pick_place.py'

def main():
    #TODO
    # You must implement a Single-Node architecture where all perception and actuation modules share the same global Executor.
    #END of TODO
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    #TODO
    #Implement the pick-and-place loop. Constraint: Use the updated TF2 coordinate convention (no leading slashes). The motion commands must account for the new moving_time parameters
    # END OF TODO
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
