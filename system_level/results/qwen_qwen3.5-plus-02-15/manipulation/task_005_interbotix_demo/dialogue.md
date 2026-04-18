# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: pick_and_place.py
----------------------------
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

----------------------------


# LLM Response

[FILENAME: pick_and_place.py]
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

# This script uses a color/depth camera to get the arm to find objects and pick them up.
# For this demo, the arm is placed to the left of the camera facing outward. When the
# end-effector is located at x=0, y=-0.3, z=0.2 w.r.t. the 'wx200/base_link' frame, the AR
# tag should be clearly visible to the camera. A small basket should also be placed in front of the arm.
#
# To get started, open a terminal and type 'ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py robot_model:=wx200'
# Then change to this directory and type 'python3 pick_and_place.py'

def main():
    rclpy.init(args=sys.argv)
    node = Node('pick_and_place')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    bot = InterbotixManipulatorXS(robot_model='wx200', group_name='arm', gripper_name='gripper', node_name='pick_and_place', node=node)
    armtag = InterbotixArmTagInterface(node=node)
    pc = InterbotixPointCloudInterface(node=node)

    executor.spin_once(timeout_sec=1.0)

    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=2.0)

    try:
        while rclpy.ok():
            object_poses = pc.find_objects()
            if not object_poses:
                break
            
            target_pose = object_poses[0]
            
            bot.arm.set_ee_pose_components(x=target_pose[0], y=target_pose[1], z=0.2, moving_time=2.0)
            bot.arm.set_ee_pose_components(x=target_pose[0], y=target_pose[1], z=target_pose[2], moving_time=2.0)
            bot.gripper.control(open=False)
            time.sleep(1.0)
            
            bot.arm.set_ee_pose_components(x=target_pose[0], y=target_pose[1], z=0.2, moving_time=2.0)
            bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=2.0)
            
            bot.arm.set_ee_pose_components(x=0.3, y=-0.3, z=0.1, moving_time=2.0)
            bot.gripper.control(open=True)
            time.sleep(1.0)
            bot.arm.set_ee_pose_components(x=0.3, y=-0.3, z=0.2, moving_time=2.0)
            
            break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()