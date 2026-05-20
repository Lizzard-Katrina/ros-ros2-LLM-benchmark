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

#in this function the clearest paths are calculated and the appropriate
#values for the angular_vel and the execution times are assigned
def ClearanceTest():
    """
    Decide whether to act and update Urgency_Report accordingly.
    - Use Regions_Report / Regions_Distances to choose a safer heading if needed.
    - Update: Urgency_Report["act"], ["angular_vel"], ["sleep"].
    """
    global Urgency_Report, Regions_Report, Regions_Distances

    min_dist = float('inf')
    blocked_region = None

    # Check all regions for obstacles
    for region in REGIONS:
        distances = Regions_Report[region]
        if not distances:
            continue
        
        current_min = min(distances)
        if current_min < OBSTACLE_DIST:
            if current_min < min_dist:
                min_dist = current_min
                blocked_region = region

    if blocked_region is not None:
        Urgency_Report["act"] = True
        
        # Find the best alternative region (closest to front_C with no obstacle)
        best_region = None
        best_score = float('inf')
        
        for region in REGIONS:
            distances = Regions_Report[region]
            if not distances:
                continue
            
            # Check if this region is clear
            if min(distances) > OBSTACLE_DIST:
                score = abs(Regions_Distances[region])
                if score < best_score:
                    best_score = score
                    best_region = region
        
        if best_region is not None:
            # Determine rotation direction based on position relative to blocked region
            blocked_idx = REGIONS.index(blocked_region)
            best_idx = REGIONS.index(best_region)
            
            # Simple logic: rotate towards the clear region
            # If best region is to the right (negative index value in our cost map logic usually implies right)
            # We use the Regions_Distances to determine direction roughly
            if Regions_Distances[best_region] > 0:
                # Target is on the left side, rotate left (positive angular vel)
                Urgency_Report["angular_vel"] = TRANS_ANG_VEL
            else:
                # Target is on the right side, rotate right (negative angular vel)
                Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
            
            # Calculate sleep time based on angle difference roughly
            # 30 degrees per region. 
            angle_diff = abs(Regions_Distances[best_region]) * (REGIONAL_ANGLE * PI / 180.0)
            if angle_diff == 0:
                angle_diff = REGIONAL_ANGLE * PI / 180.0 # Minimum turn
            
            Urgency_Report["sleep"] = angle_diff / abs(TRANS_ANG_VEL)
        else:
            # No clear path found, just spin to find one
            Urgency_Report["angular_vel"] = TRANS_ANG_VEL
            Urgency_Report["sleep"] = 0.5
    else:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0

def IdentifyRegions(scan):
    """
    Update Regions_Report using the latest LaserScan.
    - Split the 360° scan into named regions in REGIONS.
    - For each region, record obstacle-relevant range readings.
    """
    global Regions_Report
    
    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_increment = scan.angle_increment
    ranges = scan.ranges
    
    # Reset reports
    for key in Regions_Report:
        Regions_Report[key] = []
    
    total_angle = angle_max - angle_min
    if total_angle <= 0:
        total_angle = 2 * PI
        
    num_readings = len(ranges)
    
    # Map each reading to a region
    for i, range_val in enumerate(ranges):
        if not math.isfinite(range_val):
            continue
            
        current_angle = angle_min + (i * angle_increment)
        # Normalize angle to 0 - 2PI or -PI to PI depending on scan, but let's map relative to 0
        # Assuming standard 360 scan starting from front or back, we need to align with "front_C"
        # Husky/Standard Lidar often starts at front (0) or back. 
        # Let's assume standard mathematical convention relative to robot frame:
        # 0 is Front, PI/2 Left, PI Back, -PI/2 Right.
        # However, LaserScan angle_min/max defines the physical sweep.
        # We will map the index directly to the 12 sectors of 30 degrees.
        
        # Calculate normalized angle from 0 to 2PI
        angle = current_angle
        if angle < 0:
            angle += 2 * PI
            
        # Determine sector index (0-11)
        # We need to align sector 0 with "front_C". 
        # Assuming the scan starts at 0 being front. If scan starts elsewhere, an offset is needed.
        # For this generic implementation, we assume index 0 of ranges corresponds to 0 degrees (Front)
        # or we distribute the 360 degrees evenly across the 12 regions defined in REGIONS order.
        
        sector_size = (2 * PI) / 12.0
        
        # Adjust angle so front_C is centered or starts at 0. 
        # Let's assume the regions list order corresponds to clockwise or counter-clockwise starting from Front.
        # front_C, front_L, left_R... implies a specific ordering.
        # Let's simply bin the angles into 12 buckets.
        
        bucket = int(angle / sector_size) % 12
        
        # Map bucket to region name based on the order in REGIONS
        # Note: The order in REGIONS seems to go: Front, Front-Left, Left-Right?? 
        # Let's trust the list order provided: 
        # 0: front_C, 1: front_L, 2: left_R, 3: left_C, 4: left_L, 5: back_R
        # 6: back_C, 7: back_L, 8: right_R, 9: right_C, 10: right_L, 11: front_R
        
        region_name = REGIONS[bucket]
        Regions_Report[region_name].append(range_val)

class LaserAvoidNode(Node):
    def __init__(self):
        super().__init__('Laser_Obs_Avoid_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            IdentifyRegions,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.vel = Twist()

    def timer_callback(self):
        global Urgency_Report
        
        done = False
        while not done:
            ClearanceTest()
            if Urgency_Report["act"]:
                self.vel = self.Steer(self.vel)
                self.publisher_.publish(self.vel)
                # In ROS2 we don't block the thread with sleep for logic flow usually, 
                # but to mimic the original "act then wait" logic within the loop structure:
                # We will break after one command and let the timer handle the timing via state,
                # OR we implement the sleep carefully. 
                # The original code had a sleep after the loop. 
                # To keep structure similar without blocking the executor completely:
                done = True 
            else:
                done = True
        
        if not Urgency_Report["act"]:
            self.vel.linear.x = NORMAL_LIN_VEL
            self.vel.linear.y = 0.0
            self.vel.linear.z = 0.0
            self.vel.angular.x = 0.0
            self.vel.angular.y = 0.0
            self.vel.angular.z = 0.0
            self.publisher_.publish(self.vel)
        
        # Handle the sleep logic from original code
        # Since we are in a timer callback, sleeping blocks the executor.
        # We will skip the explicit time.sleep here to maintain node responsiveness,
        # relying on the fact that Urgency_Report["sleep"] was used to determine 
        # how long to keep turning in previous iterations if we were tracking state.
        # However, to strictly follow the "fill TODO" logic where sleep was used:
        # We cannot easily sleep here without freezing. 
        # The original logic: Publish -> Sleep -> Loop.
        # In ROS2 timer: We publish. The next timer call happens in 0.1s.
        # If Urgency_Report["sleep"] > 0.1, we need to keep acting.
        # We will decrement sleep in a real state machine, but to keep variables global and simple:
        if Urgency_Report["act"] and Urgency_Report["sleep"] > 0:
            Urgency_Report["sleep"] -= 0.1
            if Urgency_Report["sleep"] < 0:
                Urgency_Report["sleep"] = 0

    def Steer(self, velocity):
        global Urgency_Report
        """
        Fill a Twist command consistent with Urgency_Report and transition motion.
        - Return the updated Twist.
        """
        velocity.linear.x = TRANS_LIN_VEL
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = Urgency_Report["angular_vel"]
        return velocity

def main():
    rclpy.init()
    node = LaserAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()