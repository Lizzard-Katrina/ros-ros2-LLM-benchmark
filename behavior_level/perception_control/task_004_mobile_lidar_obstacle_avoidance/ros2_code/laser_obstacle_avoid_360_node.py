#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans

OBSTACLE_DIST = 0.5
REGIONAL_ANGLE = 30
PI = 3.141592653

NORMAL_LIN_VEL = 0.50
TRANS_LIN_VEL = -0.08
TRANS_ANG_VEL = 1.75

REGIONS = [
             "front_C", "front_L", "left_R",
             "left_C", "left_L", "back_R",
             "back_C", "back_L", "right_R",
             "right_C", "right_L", "front_R",
          ]

Urgency_Report = {
                    "act": False, "angular_vel": 0.0, "sleep": 0
                 }

Regions_Report = {
                     "front_C":[], "front_L":[], "left_R":[],
                     "left_C":[], "left_L":[], "back_R":[],
                     "back_C":[], "back_L":[], "right_R":[],
                     "right_C":[], "right_L":[], "front_R":[],
                 }

Regions_Distances = {
                     "front_C": 0, "front_L": 1, "left_R": 2,
                     "left_C": 3, "left_L": 4, "back_R": 5,
                     "back_C": 6, "back_L": -5, "right_R": -4,
                     "right_C": -3, "right_L": -2, "front_R": -1,
                 }

def ClearanceTest():
    # Determine if any region has obstacles closer than OBSTACLE_DIST
    # If so, decide a safer heading by choosing the region with the farthest minimum distance
    min_distances = {}
    for region in REGIONS:
        if Regions_Report[region]:
            min_distances[region] = min(Regions_Report[region])
        else:
            min_distances[region] = float('inf')

    # Check if any region has obstacle closer than threshold
    obstacle_regions = [r for r, dist in min_distances.items() if dist <= OBSTACLE_DIST]

    if not obstacle_regions:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0
        return

    # Find the region with the maximum clearance (largest min distance)
    safest_region = max(min_distances, key=min_distances.get)
    safest_distance = min_distances[safest_region]

    # Calculate angular velocity to steer away from obstacles towards safest region
    # Use Regions_Distances to determine direction and magnitude
    deviation = Regions_Distances[safest_region]
    angular_vel = TRANS_ANG_VEL * (deviation / max(abs(v) for v in Regions_Distances.values()))

    Urgency_Report["act"] = True
    Urgency_Report["angular_vel"] = angular_vel
    Urgency_Report["sleep"] = 0.1  # small sleep time to allow maneuver

def IdentifyRegions(scan):
    # Clear previous readings
    for region in REGIONS:
        Regions_Report[region] = []

    # LaserScan angle range is from scan.angle_min to scan.angle_max
    # Usually angle_min = -pi, angle_max = pi, with increments scan.angle_increment
    # We'll map each reading to a region based on its angle

    angle = scan.angle_min
    for r in scan.ranges:
        # Normalize angle to [0, 360) degrees
        deg = (angle * 180.0 / PI) % 360

        # Assign to region based on deg
        # Regions are 12 sectors of 30 degrees each starting at front_C at 0 deg
        # The order in REGIONS corresponds to sectors starting at front_C (0 deg) and moving CCW
        # front_C: -15 to 15 deg (345 to 15)
        # front_L: 15 to 45
        # left_R: 45 to 75
        # left_C: 75 to 105
        # left_L: 105 to 135
        # back_R: 135 to 165
        # back_C: 165 to 195
        # back_L: 195 to 225
        # right_R: 225 to 255
        # right_C: 255 to 285
        # right_L: 285 to 315
        # front_R: 315 to 345

        if deg >= 345 or deg < 15:
            region = "front_C"
        elif 15 <= deg < 45:
            region = "front_L"
        elif 45 <= deg < 75:
            region = "left_R"
        elif 75 <= deg < 105:
            region = "left_C"
        elif 105 <= deg < 135:
            region = "left_L"
        elif 135 <= deg < 165:
            region = "back_R"
        elif 165 <= deg < 195:
            region = "back_C"
        elif 195 <= deg < 225:
            region = "back_L"
        elif 225 <= deg < 255:
            region = "right_R"
        elif 255 <= deg < 285:
            region = "right_C"
        elif 285 <= deg < 315:
            region = "right_L"
        elif 315 <= deg < 345:
            region = "front_R"
        else:
            region = None

        if region is not None:
            # Only consider valid range readings (non-inf, non-zero)
            if r > 0.0 and r < float('inf'):
                Regions_Report[region].append(r)

        angle += scan.angle_increment

def Steer(velocity):
    global Urgency_Report

    vel = Twist()
    # If acting, back up and rotate with angular velocity from Urgency_Report
    if Urgency_Report["act"]:
        vel.linear.x = TRANS_LIN_VEL
        vel.angular.z = Urgency_Report["angular_vel"]
    else:
        vel.linear.x = NORMAL_LIN_VEL
        vel.angular.z = 0.0

    vel.linear.y = 0.0
    vel.linear.z = 0.0
    vel.angular.x = 0.0
    vel.angular.y = 0.0

    return vel

class LaserObsAvoidNode(Node):
    def __init__(self):
        super().__init__('Laser_Obs_Avoid_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.vel = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.done = True

    def scan_callback(self, msg):
        IdentifyRegions(msg)

    def timer_callback(self):
        global Urgency_Report

        self.done = False
        while not self.done:
            ClearanceTest()
            if Urgency_Report["act"]:
                self.vel = Steer(self.vel)
                self.pub.publish(self.vel)
            else:
                self.done = True

        if not Urgency_Report["act"]:
            self.vel.linear.x = NORMAL_LIN_VEL
            self.vel.linear.y = 0.0
            self.vel.linear.z = 0.0
            self.vel.angular.x = 0.0
            self.vel.angular.y = 0.0
            self.vel.angular.z = 0.0
            self.pub.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    node = LaserObsAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()