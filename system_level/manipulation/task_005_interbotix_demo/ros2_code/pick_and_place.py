import time
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.bot = InterbotixManipulatorXS('wx200')
        self.armtag = InterbotixArmTagInterface()
        self.pointcloud = InterbotixPointCloudInterface()

    def main(self):
        self.armtag.find_ref_to_arm_base_transform()
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)

        while True:
            try:
                self.armtag.find_objects()
                object_pose = self.armtag.get_object_pose()
                if object_pose:
                    self.bot.arm.pick_up(object_pose)
                    self.bot.arm.place(object_pose)
                self.bot.arm.go_to_sleep_pose()
                time.sleep(1)
            except ExternalShutdownException:
                break
            except Exception as e:
                self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    pick_and_place_node = PickAndPlaceNode()
    try:
        rclpy.spin(pick_and_place_node)
    except KeyboardInterrupt:
        pick_and_place_node.get_logger().info('Keyboard Interrupt (SIGINT)')
    except ExternalShutdownException:
        pick_and_place_node.get_logger().info('External Shutdown')
    finally:
        pick_and_place_node.destroy_node()
        rclpy.try_shutdown()

if __name__=='__main__':
    main()