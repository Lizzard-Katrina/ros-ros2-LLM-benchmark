# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python3
import rospy

# test/mocks/laser_scan_mock.py

class Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""

class LaserScan:
    def __init__(self):
        self.header = Header()

        # Angular limits
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0

        # Timing
        self.time_increment = 0.0
        self.scan_time = 0.0

        # Range limits
        self.range_min = 0.0
        self.range_max = 0.0

        # Data
        self.ranges = []
        self.intensities = []



def callback(msg):
    # TODO: Process the incoming LaserScan message
    # Example: print the closest range value

def main():
    rospy.init_node("lidar_subscriber")

    # Create a subscriber for /scan topic using LaserScan

    rospy.spin()
    # END OF TODO
if __name__ == "__main__":
    main()

----------------------------


# LLM Response

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanMock(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.callback,
            10)

    def callback(self, msg):
        # TODO: Process the incoming LaserScan message
        # Example: print the closest range value
        self.get_logger().info('Received scan with %d ranges' % len(msg.ranges))
        self.get_logger().info('Closest range: %f' % min(msg.ranges))

def main(args=None):
    rclpy.init(args=args)
    laser_scan_mock = LaserScanMock()
    rclpy.spin(laser_scan_mock)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser_scan_mock.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```