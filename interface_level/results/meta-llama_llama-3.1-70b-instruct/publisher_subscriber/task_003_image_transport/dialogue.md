# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python3
import rospy
#mock Image Class
class Image:
    # minimal mock attributes for Image message
    width = 640
    height = 480
    encoding = "rgb8"
    data = b''
def callback(msg):
    # TODO: ensure the usage of image_transport
    rospy.loginfo("Received an image")

def main():
    rospy.init_node('camera_subscriber_node')
    
    # use image_transport to construct subscriber
    # sub = ...

    rospy.spin()
    # END OF TODO
if __name__ == '__main__':
    main()

----------------------------


# LLM Response

```python
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
```