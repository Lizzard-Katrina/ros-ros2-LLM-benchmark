#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan  # ros msg that gets the laser scans

# obstacle threshold, objects at this distance or below it
# are considered obstacles
OBSTACLE_DIST = 0.5
# the angle in which each region extends
REGIONAL_ANGLE = 30
PI = 3.141592653

# when there's no obstacles, the robot will move with this linear velocity
NORMAL_LIN_VEL = 0.50  # meters/second
# after detecting an obstacle, the robot shall back up a bit (negative) while
# rotating to help in case it can't perform a stationary rotation
TRANS_LIN_VEL = -0.08
# the robot always rotates with the same value of angular velocity
TRANS_ANG_VEL = 1.75

# this list keeps track of the order in which the regions' readings are obtained
REGIONS = [
             "front_C", "front_L", "left_R",
             "left_C", "left_L", "back_R",
             "back_C", "back_L", "right_R",
             "right_C", "right_L", "front_R",
          ]

# this is a global variable that keeps handles the orders for the robot to follow
# if there's a detected object, "act" is turned to True
# and the angular_vel and sleep values are calculated appropriately
Urgency_Report = {
                    "act": False, "angular_vel": 0.0, "sleep": 0
                 }

# this dict keeps track of the distance measures for each region
Regions_Report = {
                     "front_C": [], "front_L": [], "left_R": [],
                     "left_C": [], "left_L": [], "back_R": [],
                     "back_C": [], "back_L": [], "right_R": [],
                     "right_C": [], "right_L": [], "front_R": [],
                 }

# These are the costs to deviate from each region to the goal region (front_C)
Regions_Distances = {
                     "front_C": 0, "front_L": 1, "left_R": 2,
                     "left_C": 3, "left_L": 4, "back_R": 5,
                     "back_C": 6, "back_L": -5, "right_R": -4,
                     "right_C": -3, "right_L": -2, "front_R": -1,
                 }


# in this function the clearest paths are calculated and the appropriate
# values for the angular_vel and the execution times are assigned
def ClearanceTest():
    global Urgency_Report

    goal = "front_C"
    closest = 10e6
    regional_dist = 0
    maxima = {"destination": "back_C", "distance": 10e-6}
    for region in Regions_Report.items():
        regional_dist = abs(Regions_Distances[region[0]] - Regions_Distances[goal])
        # if there're no obstacles in that region
        if not len(region[1]):
            # check if it's the cheapest option
            if regional_dist < closest:
                closest = regional_dist
                maxima["distance"] = OBSTACLE_DIST
                maxima["destination"] = region[0]
        # check if it's the clearest option
        elif max(region[1]) > maxima["distance"]:
            maxima["distance"] = max(region[1])
            maxima["destination"] = region[0]

    # calculate the cost to the chosen orientation
    regional_dist = Regions_Distances[maxima["destination"]] - Regions_Distances[goal]

    # we act whenever the clearest path is not the front_C (front center)
    Urgency_Report["act"] = (closest != 0)
    Urgency_Report["angular_vel"] = ((regional_dist / max(1, abs(regional_dist)))
                                     * TRANS_ANG_VEL)
    Urgency_Report["sleep"] = ((abs(regional_dist) * REGIONAL_ANGLE * PI)
                               / (180 * TRANS_ANG_VEL))


def IdentifyRegions(scan):
    global Regions_Report
    for i, region in enumerate(REGIONS):
        Regions_Report[region] = [
                x for x in scan.ranges[REGIONAL_ANGLE * i: REGIONAL_ANGLE * (i + 1)]
                        if x <= OBSTACLE_DIST and x != float('inf')]


def Steer(velocity):
    global Urgency_Report

    # since we're moving only on the plane, all we need is move in the x axis,
    # and rotate in the z (zeta) axis.
    velocity.linear.x = TRANS_LIN_VEL
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = Urgency_Report["angular_vel"]

    return velocity


class LaserObsAvoidNode(Node):
    def __init__(self):
        super().__init__('Laser_Obs_Avoid_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(LaserScan, '/scan', IdentifyRegions, 10)
        self.vel = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Need a do{ ... }while(); here
        # Since I need to check at least once the clearance
        done = False
        while not done:
            ClearanceTest()
            if(Urgency_Report["act"]):
                self.vel = Steer(self.vel)
                self.pub.publish(self.vel)
            else:
                done = True
        # This else belongs to the while(), and the code below it could be cleaned furthermore
        else:
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()