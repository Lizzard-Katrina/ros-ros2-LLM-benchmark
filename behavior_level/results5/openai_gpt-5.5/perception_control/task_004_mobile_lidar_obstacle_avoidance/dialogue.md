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

FILE_PATH: laser_obstacle_avoid_360_node.py
----------------------------
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
# import time

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
    """
    TODO: Decide whether to act and update Urgency_Report accordingly.
    - Use Regions_Report / Regions_Distances to choose a safer heading if needed.
    - Update: Urgency_Report["act"], ["angular_vel"], ["sleep"].
    END OF TODO
    """

   
def IdentifyRegions(scan):
    """
    TODO: Update Regions_Report using the latest LaserScan.
    - Split the 360° scan into named regions in REGIONS.
    - For each region, record obstacle-relevant range readings.
    END OF TODO
    """

def Steer(velocity):
    global Urgency_Report

    """
    TODO: Fill a Twist command consistent with Urgency_Report and transition motion.
    - Return the updated Twist.
    END OF TODO
    """
def main():
    #Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    #Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, IdentifyRegions)
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    vel = Twist()
    #ros will try to run this code 10 times/second
    rate = rospy.Rate(10) #10Hz
    #keep running while the ros-master isn't isn't shutdown
    while not rospy.is_shutdown():

        # Need a do{ ... }while(); here (C is awesome)
        # Since I need to check at least once the clearance 
        done = False
        while not done:
            ClearanceTest()
            if(Urgency_Report["act"]):
                vel = Steer(vel)
                pub.publish(vel)
            else:
                done = True
        # This else belongs to the while(), and the code below it could be cleaned furthermore
        else: 
            vel.linear.x = NORMAL_LIN_VEL
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            pub.publish(vel)

        ### This is stupid and shouldn't be done (sleep()) !
        # After publishing our action, we give it some time to execute the
        # needed actions before reading the data again.
        # time.sleep(Urgency_Report["sleep"])
        
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

----------------------------


# LLM Response

[FILENAME: laser_obstacle_avoid_360_node.py]
#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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

_node = None

#in this function the clearest paths are calculated and the appropriate
#values for the angular_vel and the execution times are assigned
def ClearanceTest():
    global Urgency_Report
    global Regions_Report

    front_blocked = (
        len(Regions_Report["front_C"]) > 0 or
        len(Regions_Report["front_L"]) > 0 or
        len(Regions_Report["front_R"]) > 0
    )

    any_obstacle = any(len(readings) > 0 for readings in Regions_Report.values())

    if not any_obstacle or not front_blocked:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0
        return

    clear_regions = [region for region in REGIONS if len(Regions_Report[region]) == 0]

    if clear_regions:
        best_region = min(clear_regions, key=lambda region: abs(Regions_Distances[region]))
        distance = Regions_Distances[best_region]
    else:
        obstacle_scores = {}
        for region in REGIONS:
            readings = Regions_Report[region]
            if readings:
                obstacle_scores[region] = min(readings)
            else:
                obstacle_scores[region] = float("inf")
        best_region = max(REGIONS, key=lambda region: (obstacle_scores[region], -abs(Regions_Distances[region])))
        distance = Regions_Distances[best_region]

    if distance == 0:
        if len(Regions_Report["front_L"]) > len(Regions_Report["front_R"]):
            distance = -1
        elif len(Regions_Report["front_R"]) > len(Regions_Report["front_L"]):
            distance = 1
        else:
            left_obstacles = sum(len(Regions_Report[region]) for region in ["front_L", "left_R", "left_C", "left_L"])
            right_obstacles = sum(len(Regions_Report[region]) for region in ["front_R", "right_L", "right_C", "right_R"])
            distance = -1 if left_obstacles >= right_obstacles else 1

    Urgency_Report["act"] = True
    Urgency_Report["angular_vel"] = TRANS_ANG_VEL if distance > 0 else -TRANS_ANG_VEL
    Urgency_Report["sleep"] = max(1, min(6, abs(distance)))

   
def IdentifyRegions(scan):
    global Regions_Report

    for region in REGIONS:
        Regions_Report[region] = []

    if scan is None or not scan.ranges:
        return

    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if distance < scan.range_min or distance > scan.range_max:
            continue
        if distance > OBSTACLE_DIST:
            continue

        angle = scan.angle_min + index * scan.angle_increment
        angle_deg = math.degrees(angle)

        while angle_deg < 0.0:
            angle_deg += 360.0
        while angle_deg >= 360.0:
            angle_deg -= 360.0

        shifted_angle = (angle_deg + REGIONAL_ANGLE / 2.0) % 360.0
        region_index = int(shifted_angle // REGIONAL_ANGLE) % len(REGIONS)
        Regions_Report[REGIONS[region_index]].append(distance)

    ClearanceTest()

def Steer(velocity):
    global Urgency_Report

    velocity.linear.x = TRANS_LIN_VEL
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = Urgency_Report["angular_vel"]

    if Urgency_Report["sleep"] > 0:
        time.sleep(float(Urgency_Report["sleep"]) * 0.1)

    return velocity

def main():
    global _node

    rclpy.init()
    #Initialize our node
    _node = Node("Laser_Obs_Avoid_node")
    #Subscribe to the "/scan" topic in order to read laser scans data from it
    _node.create_subscription(LaserScan, "/scan", IdentifyRegions, 10)
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = _node.create_publisher(Twist, "/cmd_vel", 1)
    vel = Twist()
    #ros will try to run this code 10 times/second
    rate = _node.create_rate(10) #10Hz
    #keep running while the ros-master isn't isn't shutdown
    try:
        while rclpy.ok():

            rclpy.spin_once(_node, timeout_sec=0.0)

            # Need a do{ ... }while(); here (C is awesome)
            # Since I need to check at least once the clearance 
            done = False
            while not done and rclpy.ok():
                ClearanceTest()
                if(Urgency_Report["act"]):
                    vel = Steer(vel)
                    pub.publish(vel)
                    done = True
                else:
                    done = True
            # This else belongs to the while(), and the code below it could be cleaned furthermore
            else: 
                if rclpy.ok() and not Urgency_Report["act"]:
                    vel.linear.x = NORMAL_LIN_VEL
                    vel.linear.y = 0.0
                    vel.linear.z = 0.0
                    vel.angular.x = 0.0
                    vel.angular.y = 0.0
                    vel.angular.z = 0.0
                    pub.publish(vel)

            ### This is stupid and shouldn't be done (sleep()) !
            # After publishing our action, we give it some time to execute the
            # needed actions before reading the data again.
            # time.sleep(Urgency_Report["sleep"])
            
            rate.sleep()
    finally:
        if _node is not None:
            _node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
