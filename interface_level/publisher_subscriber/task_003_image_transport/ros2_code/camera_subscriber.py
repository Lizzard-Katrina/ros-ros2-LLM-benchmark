#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        self.get_logger().info("Camera Subscriber Node Started")
        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.callback,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup())

    def callback(self, msg):
        self.get_logger().info("Received an image")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraSubscriberNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')
        except ExternalShutdownException:
            node.get_logger().info('External Shutdown')
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    except Exception as e:
        print(f"Error occurred: {e}")

if __name__ == '__main__':
    main()