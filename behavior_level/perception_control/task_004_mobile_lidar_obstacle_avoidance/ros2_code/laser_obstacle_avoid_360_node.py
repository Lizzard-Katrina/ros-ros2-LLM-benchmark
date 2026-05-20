#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time

# obstacle threshhold, objects a this distance or below it
#are considered obstacles
OBSTACLE_DIST = 0.5
#the angle in which each region extends
REGIONAL_ANGLE = 30
PI = 3.141592653

#when there's no obstacles, the robot will move with this linear velocity
NORMAL_LIN_VEL = 0.50 #meters/second
#after detecting an obstacle, the robot shall back up a bit (negative) while
# rotating to help in case it can't perform a stationary rotation
TRANS_LIN_VEL = -0.08
#the robot always rotates with the same value of angular velocity
TRANS_ANG_VEL = 1.75

#this list keeps track of the order in which the regions' readings are obtained
REGIONS = [
             "front_C", "front_L", "left_R",
             "left_C", "left_L", "back_R",
             "back_C", "back_L", "right_R",
             "right_C", "right_L", "front_R",
          ]
#this is a global variable that keeps handles the orders for the robot to follow
#if there's a detected object, "act" is turned to True
#and the angular_vel and sleep values are calculated appropriately
Urgency_Report = {
                    "act": False, "angular_vel": 0.0, "sleep": 0
                 }
#this dict keeps track of the distance measures for each region
Regions_Report = {
                     "front_C":[], "front_L":[], "left_R":[],
                     "left_C":[], "left_L":[], "back_R":[],
                     "back_C":[], "back_L":[], "right_R":[],
                     "right_C":[], "right_L":[], "front_R":[],
                 }
#These are the costs to deviate from each region to the goal region (front_C)
Regions_Distances = {
                     "front_C": 0, "front_L": 1, "left_R": 2,
                     "left_C": 3, "left_L": 4, "back_R": 5,
                     "back_C": 6, "back_L": -5, "right_R": -4,
                     "right_C": -3, "right_L": -2, "front_R": -1,
                 }

#in this function the clearest paths are calculated and the appropriate
#values for the angular_vel and the execution times are assigned
def ClearanceTest():
    global Urgency_Report, Regions_Report, Regions_Distances
    
    front_clear = True
    if Regions_Report["front_C"]:
        for dist in Regions_Report["front_C"]:
            if dist < OBSTACLE_DIST:
                front_clear = False
                break
                
    if front_clear:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0
        return

    best_region = "front_C"
    max_clearance = 0.0
    
    for region, distances in Regions_Report.items():
        if not distances:
            continue
        min_dist = min(distances)
        if min_dist > max_clearance:
            max_clearance = min_dist
            best_region = region
            
    Urgency_Report["act"] = True
    dist_cost = Regions_Distances[best_region]
    if dist_cost > 0:
        Urgency_Report["angular_vel"] = TRANS_ANG_VEL
    else:
        Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
        
    Urgency_Report["sleep"] = abs(dist_cost) * 0.1

   
def IdentifyRegions(scan):
    global Regions_Report
    
    ranges = scan.ranges
    num_ranges = len(ranges)
    if num_ranges == 0:
        return
        
    region_size = num_ranges // len(REGIONS)
    
    for i, region in enumerate(REGIONS):
        start_idx = i * region_size
        end_idx = start_idx + region_size
        
        region_ranges = ranges[start_idx:end_idx]
        valid_ranges = [r for r in region_ranges if not math.isinf(r) and not math.isnan(r)]
        if not valid_ranges:
            valid_ranges = [float('inf')]
            
        Regions_Report[region] = valid_ranges

def Steer(velocity):
    global Urgency_Report

    velocity.linear.x = TRANS_LIN_VEL
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = Urgency_Report["angular_vel"]
    
    return velocity

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("Laser_Obs_Avoid_node")
    
    node.create_subscription(LaserScan, "/scan", IdentifyRegions, 10)
    pub = node.create_publisher(Twist, "/cmd_vel", 1)
    vel = Twist()
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        
        done = False
        while not done:
            ClearanceTest()
            if(Urgency_Report["act"]):
                vel = Steer(vel)
                pub.publish(vel)
                time.sleep(Urgency_Report["sleep"])
                rclpy.spin_once(node, timeout_sec=0.0)
            else:
                done = True
        else: 
            vel.linear.x = NORMAL_LIN_VEL
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 0.0
            pub.publish(vel)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass