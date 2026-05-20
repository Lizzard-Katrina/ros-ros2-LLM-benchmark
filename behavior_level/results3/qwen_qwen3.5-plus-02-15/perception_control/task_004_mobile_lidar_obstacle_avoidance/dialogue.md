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

    Urgency_Report["act"] = False
    Urgency_Report["angular_vel"] = 0.0
    Urgency_Report["sleep"] = 0

    # Calculate minimum distance for each region
    region_mins = {}
    obstacle_detected = False

    for region in REGIONS:
        distances = Regions_Report[region]
        if distances:
            min_dist = min(distances)
            region_mins[region] = min_dist
            if min_dist <= OBSTACLE_DIST:
                obstacle_detected = True
        else:
            region_mins[region] = float('inf')

    if not obstacle_detected:
        return

    Urgency_Report["act"] = True

    # Find the safest region (furthest from obstacles) among those with valid data
    best_region = None
    max_dist = -1.0

    for region in REGIONS:
        dist = region_mins[region]
        if dist > max_dist:
            max_dist = dist
            best_region = region

    if best_region:
        # Determine rotation direction and magnitude based on cost to return to front_C
        cost = Regions_Distances[best_region]
        
        # If cost is positive, obstacle is on left side (need to turn right/negative)
        # If cost is negative, obstacle is on right side (need to turn left/positive)
        # Magnitude depends on how far off-center the clear path is
        
        if cost > 0:
            Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
        elif cost < 0:
            Urgency_Report["angular_vel"] = TRANS_ANG_VEL
        else:
            # Directly in front, choose a default turn (e.g., left)
            Urgency_Report["angular_vel"] = TRANS_ANG_VEL

        # Calculate sleep time based on how much we need to turn
        # Approximate: 30 degrees per region step. 
        # Time = Angle / Angular Velocity
        steps_needed = abs(cost)
        angle_to_turn = steps_needed * (REGIONAL_ANGLE * PI / 180.0)
        if angle_to_turn == 0:
            angle_to_turn = (REGIONAL_ANGLE * PI / 180.0) # Minimum turn if stuck
            
        Urgency_Report["sleep"] = angle_to_turn / abs(Urgency_Report["angular_vel"])


def IdentifyRegions(scan):
    """
    Update Regions_Report using the latest LaserScan.
    - Split the 360° scan into named regions in REGIONS.
    - For each region, record obstacle-relevant range readings.
    """
    global Regions_Report, REGIONS, REGIONAL_ANGLE, PI

    # Reset reports
    for region in REGIONS:
        Regions_Report[region] = []

    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_increment = scan.angle_increment
    ranges = scan.ranges

    # Total span
    total_angle = angle_max - angle_min
    if total_angle <= 0:
        total_angle = 2 * PI

    # We have 12 regions, each 30 degrees. 
    # We map the scan angles to these regions.
    # Assuming scan covers roughly 360 degrees (-PI to PI or 0 to 2PI)
    
    # Normalize start angle to -PI for easier calculation if needed, 
    # but let's just iterate based on index and increment.
    
    current_angle = angle_min
    
    # Map each range reading to a region
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
            
        # Calculate angle of this reading relative to robot frame (assuming scan aligns with robot)
        # ROS LaserScan: angle_min is start, angle_max is end.
        angle = angle_min + (i * angle_increment)
        
        # Normalize angle to -PI to PI
        while angle > PI:
            angle -= 2 * PI
        while angle <= -PI:
            angle += 2 * PI
            
        # Convert to degrees for easier region mapping (0 is front, 90 left, -90 right, 180 back)
        # But our regions are named specifically. Let's define boundaries.
        # front_C: -15 to 15
        # front_L: 15 to 45
        # left_R: 45 to 75
        # left_C: 75 to 105
        # left_L: 105 to 135
        # back_R: 135 to 165 (or -135 to -165?) -> Let's assume standard CCW from front
        
        # Let's map degrees:
        deg = angle * 180.0 / PI
        
        region_name = None
        
        if -15 <= deg < 15:
            region_name = "front_C"
        elif 15 <= deg < 45:
            region_name = "front_L"
        elif 45 <= deg < 75:
            region_name = "left_R"
        elif 75 <= deg < 105:
            region_name = "left_C"
        elif 105 <= deg < 135:
            region_name = "left_L"
        elif 135 <= deg <= 180 or -180 <= deg < -135:
            # Back centerish. Split back?
            # The list has back_R, back_C, back_L.
            # Standard: 180 is back. 
            # back_R: 135 to 165? back_C: 165 to -165? back_L: -165 to -135?
            # Let's refine based on the 12 regions of 30 deg.
            # 0: front_C (-15, 15)
            # 1: front_L (15, 45)
            # 2: left_R (45, 75)
            # 3: left_C (75, 105)
            # 4: left_L (105, 135)
            # 5: back_R (135, 165) -- Wait, usually Right is negative Y, Left is positive Y.
            # If robot faces X, Left is +Y (90 deg), Right is -Y (-90 deg).
            # So 135 is Back-Left? 
            # Let's re-evaluate the names vs standard math angles.
            # front_L (Front-Left) -> Positive angle.
            # left_R (Left-Rear?) or Left-Right? Usually "Left Front", "Left Side", "Left Rear".
            # Given the list order: front_C, front_L, left_R, left_C, left_L...
            # It seems to go Counter-Clockwise starting from Front.
            # front_C (0)
            # front_L (30)
            # left_R (60) -> Maybe "Left Region 1"?
            # left_C (90)
            # left_L (120)
            # back_R (150) -> "Back Region 1"?
            # back_C (180)
            # back_L (210/-150)
            # right_R (240/-120)
            # right_C (270/-90)
            # right_L (300/-60)
            # front_R (330/-30)
            
            # Let's assume the names are just labels for 30-degree slices CCW.
            # Slice 0: -15 to 15 -> front_C
            # Slice 1: 15 to 45 -> front_L
            # Slice 2: 45 to 75 -> left_R
            # Slice 3: 75 to 105 -> left_C
            # Slice 4: 105 to 135 -> left_L
            # Slice 5: 135 to 165 -> back_R (Label might be confusing but follows sequence)
            # Slice 6: 165 to 195 (-165 to -165) -> back_C
            # Slice 7: 195 to 225 (-165 to -135) -> back_L
            # Slice 8: 225 to 255 (-135 to -105) -> right_R
            # Slice 9: 255 to 285 (-105 to -75) -> right_C
            # Slice 10: 285 to 315 (-75 to -45) -> right_L
            # Slice 11: 315 to 345 (-45 to -15) -> front_R
            
            if 135 <= deg <= 165:
                region_name = "back_R"
            elif 165 < deg <= 180 or -180 <= deg < -165:
                region_name = "back_C" # Covering the seam
            elif -165 <= deg < -135:
                region_name = "back_L"
            elif -135 <= deg < -105:
                region_name = "right_R"
            elif -105 <= deg < -75:
                region_name = "right_C"
            elif -75 <= deg < -45:
                region_name = "right_L"
            elif -45 <= deg < -15:
                region_name = "front_R"
        
        # Handle the specific back_C seam case more robustly if not caught
        if region_name is None and (deg > 165 or deg < -165):
             region_name = "back_C"

        if region_name:
            Regions_Report[region_name].append(r)

def Steer(velocity):
    global Urgency_Report

    """
    Fill a Twist command consistent with Urgency_Report and transition motion.
    - Return the updated Twist.
    """
    if Urgency_Report["act"]:
        velocity.linear.x = TRANS_LIN_VEL
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = Urgency_Report["angular_vel"]
    else:
        # Should not be called if not acting, but safe guard
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        
    return velocity

class LaserObsAvoidNode(Node):
    def __init__(self):
        super().__init__("Laser_Obs_Avoid_node")
        
        self.sub = self.create_subscription(LaserScan, "/scan", IdentifyRegions, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.vel = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz

    def timer_callback(self):
        # Need a do{ ... }while(); here (C is awesome)
        # Since I need to check at least once the clearance 
        done = False
        while not done:
            ClearanceTest()
            if(Urgency_Report["act"]):
                self.vel = Steer(self.vel)
                self.pub.publish(self.vel)
                
                # Simulate the sleep logic from ROS1 by blocking briefly or just setting velocity
                # In ROS2 timer, we can't sleep effectively without stopping other callbacks.
                # However, the original code slept AFTER publishing.
                # To mimic "give it time to execute", we rely on the loop running at 10Hz.
                # If sleep was 0.5s, we'd need to maintain velocity for 5 cycles.
                # The original code structure: Publish -> Sleep -> Loop.
                # Here: Publish. Next timer call happens in 0.1s.
                # If Urgency_Report["sleep"] > 0.1, we should keep acting.
                # The original code set "act" based on scan, then slept.
                # We will just publish the velocity. The ClearanceTest resets "act" every time.
                # So if we don't detect obstacle in next scan, "act" becomes false.
                # But the original code SLEPT before reading again.
                # To replicate: We need a state to hold the "act" decision for the duration of sleep.
                # However, strictly following the provided structure and TODOs:
                # The original loop: Check -> Act/Publish -> Sleep -> Check again.
                # With 10Hz rate and variable sleep, the logic is tightly coupled.
                # Let's assume the "sleep" value in Urgency_Report is used to determine 
                # if we should FORCE act in subsequent cycles even if scan clears?
                # No, the original code:
                # 1. ClearanceTest (sets act=True, sleep=X)
                # 2. Publish
                # 3. Sleep(X)
                # 4. Loop (ClearanceTest runs again on NEW scan data)
                # So the sleep just delays the next sensor read.
                # In ROS2, we can't block the timer easily.
                # We will just publish the velocity. The robot will move.
                # The "sleep" logic is effectively handled by the fact that we keep publishing 
                # the same velocity if the obstacle persists, or we rely on the robot dynamics.
                # To strictly follow "fill missing code", we just publish.
                
            else:
                done = True
        
        if not Urgency_Report["act"]:
            self.vel.linear.x = NORMAL_LIN_VEL
            self.vel.linear.y = 0.0
            self.vel.linear.z = 0.0
            self.vel.angular.x = 0.0
            self.vel.angular.y = 0.0
            self.vel.angular.z = 0.0
            self.pub.publish(self.vel)

def main():
    rclpy.init()
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