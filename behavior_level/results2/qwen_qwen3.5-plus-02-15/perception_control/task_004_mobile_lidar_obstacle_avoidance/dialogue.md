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
    best_region = None

    # Check all regions for obstacles and find the safest one
    for region in REGIONS:
        distances = Regions_Report[region]
        if not distances:
            continue
        
        current_min = min(distances)
        if current_min < OBSTACLE_DIST:
            # Obstacle detected in this region
            # We want to find a region that is clear (min_dist > OBSTACLE_DIST)
            # or the one with the largest distance if all are blocked
            pass
        
        # Track the region with the maximum minimum distance (safest path)
        if current_min > min_dist:
            min_dist = current_min
            best_region = region

    # Determine if action is needed
    # Action is needed if the front center has an obstacle or if any critical front region is blocked
    front_clear = True
    if Regions_Report["front_C"] and min(Regions_Report["front_C"]) <= OBSTACLE_DIST:
        front_clear = False
    if Regions_Report["front_L"] and min(Regions_Report["front_L"]) <= OBSTACLE_DIST:
        front_clear = False
    if Regions_Report["front_R"] and min(Regions_Report["front_R"]) <= OBSTACLE_DIST:
        front_clear = False

    if not front_clear:
        Urgency_Report["act"] = True
        
        if best_region:
            # Calculate angular velocity based on the cost to turn to the best region
            cost = Regions_Distances[best_region]
            # Positive cost means turn left (positive angular vel), negative means right
            # Magnitude depends on how far away the region is (30 degrees per step)
            Urgency_Report["angular_vel"] = TRANS_ANG_VEL * (1 if cost > 0 else -1)
            
            # Calculate sleep time based on the angle to rotate
            # Each step is REGIONAL_ANGLE degrees. Convert to radians and divide by angular velocity
            steps_to_turn = abs(cost)
            angle_to_turn_rad = math.radians(steps_to_turn * REGIONAL_ANGLE)
            Urgency_Report["sleep"] = angle_to_turn_rad / abs(TRANS_ANG_VEL)
        else:
            # Fallback if no data: spin in place
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

    # Total angle coverage
    total_angle = angle_max - angle_min
    
    # Calculate start and end indices for each region
    # Assuming standard 360 scan or similar coverage centered or starting at 0
    # We map the 12 regions evenly across the available scan or based on specific angles
    # The original logic implies 12 regions of 30 degrees each = 360 degrees.
    
    # Let's map angles to regions. 
    # We assume the scan covers -PI to PI or 0 to 2PI. 
    # Let's normalize angles to -PI to PI for easier calculation relative to robot frame
    # Front is 0, Left is +PI/2, Back is +/- PI, Right is -PI/2
    
    region_angles = {
        "front_C": (-15, 15),
        "front_L": (15, 45),
        "left_R": (45, 75),
        "left_C": (75, 105),
        "left_L": (105, 135),
        "back_R": (135, 165),
        "back_C": (165, 180), # Handling wrap around
        "back_L": (-180, -165),
        "right_R": (-165, -135),
        "right_C": (-135, -105),
        "right_L": (-105, -75),
        "front_R": (-75, -15),
    }

    # Helper to normalize angle to -180, 180
    def normalize_angle_deg(angle_deg):
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg <= -180:
            angle_deg += 360
        return angle_deg

    for i, range_val in enumerate(ranges):
        if not math.isfinite(range_val):
            continue
            
        # Calculate angle for this reading in degrees
        current_angle_rad = angle_min + (i * angle_increment)
        current_angle_deg = math.degrees(current_angle_rad)
        current_angle_deg = normalize_angle_deg(current_angle_deg)

        for region_name, (start_deg, end_deg) in region_angles.items():
            # Handle wrap-around for back regions if necessary, though ranges above cover -180 to 180
            if start_deg < end_deg:
                if start_deg <= current_angle_deg <= end_deg:
                    Regions_Report[region_name].append(range_val)
            else:
                # Wrap around case (e.g., back_C crossing 180/-180 boundary if defined differently)
                # Our definition splits back into + and - so this might not trigger often with current defs
                if current_angle_deg >= start_deg or current_angle_deg <= end_deg:
                    Regions_Report[region_name].append(range_val)

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
        velocity.linear.x = NORMAL_LIN_VEL
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
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
        done = False
        while not done:
            ClearanceTest()
            if Urgency_Report["act"]:
                self.vel = Steer(self.vel)
                self.pub.publish(self.vel)
                # Simulate the sleep logic from ROS1 by breaking or handling timing
                # In ROS2 timer, we can't block. We rely on the timer frequency.
                # The original code slept based on calculation. 
                # Here we just issue the command. The next tick will re-evaluate.
                # To mimic the "sleep" duration logic without blocking, we could use a state machine,
                # but sticking to the structure: we publish and let the loop continue next tick.
                # The original 'sleep' was to let the rotation happen before re-scanning.
                # With 10Hz, 0.1s per tick, if sleep was 1s, we'd need to ignore scans for 10 ticks.
                # For simplicity in this migration keeping structure: we just publish.
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