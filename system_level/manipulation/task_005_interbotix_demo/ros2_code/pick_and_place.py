import time
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
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
    bot = InterbotixManipulatorXS(
        robot_model='wx200',
        group_name='arm',
        gripper_name='gripper'
    )
    
    armtag = InterbotixArmTagInterface(
        core=bot.core,
        armtag_ns='wx200',
        apriltag_ns='wx200'
    )
    
    pcl = InterbotixPointCloudInterface(
        core=bot.core,
        filter_ns='wx200'
    )
    
    bot.arm.set_ee_pose_components(x=0, y=-0.3, z=0.2)
    time.sleep(0.5)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    success, clusters = pcl.get_cluster_positions(
        ref_frame='wx200/base_link',
        sort_axis='y',
        reverse=True
    )
    
    if success:
        for cluster in clusters:
            x, y, z = cluster['position']
            bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=1.5, moving_time=1.5)
            bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=1.5, moving_time=1.0)
            bot.gripper.grasp()
            bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=1.5, moving_time=1.0)
            bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
            bot.gripper.release()

    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()