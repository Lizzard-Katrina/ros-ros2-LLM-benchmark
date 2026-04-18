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
from sensor_msgs.msg import LaserScan as SensorLaserScan

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
    # Process the incoming LaserScan message
    # Example: print the closest range value
    if msg.ranges:
        closest_range = min(msg.ranges)
        print(f"Closest range: {closest_range}")
    else:
        print("No range data available")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("lidar_subscriber")

    # Create a subscriber for /scan topic using LaserScan
    subscriber = node.create_subscription(
        SensorLaserScan,
        "/scan",
        callback,
        10
    )

    rclpy.spin(node)
    # END OF TODO
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```