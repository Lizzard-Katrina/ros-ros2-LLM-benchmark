# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

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

def get_region_from_angle(angle):
    """Helper to map angle to region index based on 30 degree sectors."""
    # Normalize angle to -PI to PI
    while angle > PI:
        angle -= 2 * PI
    while angle <= -PI:
        angle += 2 * PI
    
    # Convert to degrees for easier mapping based on REGIONAL_ANGLE (30)
    deg = math.degrees(angle)
    
    # Define sectors centered at 0, 30, 60...
    # Sector 0 (front_C): -15 to 15
    # Sector 1 (front_L): 15 to 45
    # ...
    # Sector 11 (front_R): -45 to -15
    
    index = int(round((deg + 15) / 30)) % 12
    return index

#in this function the clearest paths are calculated and the appropriate
#values for the angular_vel and the execution times are assigned
def ClearanceTest():
    """
    TODO: Decide whether to act and update Urgency_Report accordingly.
    - Use Regions_Report / Regions_Distances to choose a safer heading if needed.
    - Update: Urgency_Report["act"], ["angular_vel"], ["sleep"].
    END OF TODO
    """
    obstacle_detected = False
    min_dist_per_region = {}

    # Extract latest min distance for each region
    for region in REGIONS:
        data = Regions_Report[region]
        if data:
            min_dist_per_region[region] = min(data)
        else:
            min_dist_per_region[region] = float('inf')

    # Check for obstacles
    for region, dist in min_dist_per_region.items():
        if dist < OBSTACLE_DIST:
            obstacle_detected = True
            break
    
    if obstacle_detected:
        Urgency_Report["act"] = True
        # Find safest region (distance > OBSTACLE_DIST) with lowest cost (closest to front_C)
        best_region = None
        best_cost = float('inf')
        
        for region, dist in min_dist_per_region.items():
            if dist > OBSTACLE_DIST:
                cost = abs(Regions_Distances[region])
                if cost < best_cost:
                    best_cost = cost
                    best_region = region
        
        # If no clear region found, default to rotating away from closest obstacle or just spin
        if best_region is None:
            # Fallback: rotate right
            best_region = "right_C"
            Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
        else:
            # Determine rotation direction based on region cost sign
            cost = Regions_Distances[best_region]
            if cost > 0:
                # Target is on the Left, rotate Left (positive)
                Urgency_Report["angular_vel"] = TRANS_ANG_VEL
            elif cost < 0:
                # Target is on the Right, rotate Right (negative)
                Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
            else:
                # Target is Front, but obstacle detected elsewhere? 
                # If front is clear but act is True, obstacle is on side. 
                # Continue forward or slight adjust. Let's stop rotation if front is clear.
                Urgency_Report["angular_vel"] = 0.0

        # Estimate sleep time for rotation step
        Urgency_Report["sleep"] = 0.5
    else:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0

   
def IdentifyRegions(scan):
    """
    TODO: Update Regions_Report using the latest LaserScan.
    - Split the 360° scan into named regions in REGIONS.
    - For each region, record obstacle-relevant range readings.
    END OF TODO
    """
    # Clear previous data to keep only latest scan info
    for region in REGIONS:
        Regions_Report[region] = []
    
    ranges = scan.ranges
    count = len(ranges)
    
    for i, r in enumerate(ranges):
        if math.isinf(r) or math.isnan(r):
            continue
            
        angle = scan.angle_min + i * scan.angle_increment
        idx = get_region_from_angle(angle)
        
        if 0 <= idx < len(REGIONS):
            region_name = REGIONS[idx]
            Regions_Report[region_name].append(r)

def Steer(velocity):
    global Urgency_Report

    """
    TODO: Fill a Twist command consistent with Urgency_Report and transition motion.
    - Return the updated Twist.
    END OF TODO
    """
    if Urgency_Report["act"]:
        velocity.linear.x = TRANS_LIN_VEL
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = Urgency_Report["angular_vel"]
    else:
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        
    return velocity

def main():
    #Initialize our node
    rclpy.init()
    node = Node("Laser_Obs_Avoid_node")
    
    #Subscribe to the "/scan" topic in order to read laser scans data from it
    node.create_subscription(LaserScan, "/scan", IdentifyRegions, 10)
    
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    
    vel = Twist()
    #ros will try to run this code 10 times/second
    rate = node.create_rate(10) #10Hz
    
    #keep running while the ros-master isn't isn't shutdown
    while rclpy.ok():

        # Need a do{ ... }while(); here (C is awesome)
        # Since I need to check at least once the clearance 
        done = False
        while not done:
            # Process callbacks to update Regions_Report
            rclpy.spin_once(node, timeout_sec=0.0)
            
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

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass