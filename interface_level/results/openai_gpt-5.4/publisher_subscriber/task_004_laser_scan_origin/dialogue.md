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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

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


_node = None


def callback(msg):
    if hasattr(msg, "ranges") and msg.ranges:
        valid_ranges = [r for r in msg.ranges if r is not None]
        if valid_ranges:
            closest = min(valid_ranges)
            if _node is not None:
                _node.get_logger().info(f"Closest range: {closest}")
            else:
                print(f"Closest range: {closest}")


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            callback,
            10
        )


def main():
    global _node
    rclpy.init()
    _node = LidarSubscriber()
    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()