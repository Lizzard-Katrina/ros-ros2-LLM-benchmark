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

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
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
    TODO: Decide whether to act and update Urgency_Report accordingly.
    - Use Regions_Report / Regions_Distances to choose a safer heading if needed.
    - Update: Urgency_Report["act"], ["angular_vel"], ["sleep"].
    END OF TODO
    """
    global Urgency_Report, Regions_Report, Regions_Distances

    min_dist_in_regions = {region: min(Regions_Report[region]) if Regions_Report[region] else float('inf') for region in REGIONS}

    # Check for immediate obstacles
    obstacle_detected = False
    for region in REGIONS:
        if min_dist_in_regions[region] < OBSTACLE_DIST:
            obstacle_detected = True
            break

    if not obstacle_detected:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0
        return

    Urgency_Report["act"] = True

    # Find the safest region (largest min distance)
    safest_region = None
    max_safe_dist = -1.0
    for region in REGIONS:
        if min_dist_in_regions[region] > max_safe_dist:
            max_safe_dist = min_dist_in_regions[region]
            safest_region = region
    
    # If all regions have obstacles, prioritize backing up and rotating
    if max_safe_dist < OBSTACLE_DIST:
        # Default to turning right if completely surrounded or no clear path
        Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
        Urgency_Report["sleep"] = abs(REGIONAL_ANGLE * math.pi / 180 / TRANS_ANG_VEL) # Estimate time to turn one region
        return

    # Determine angular velocity based on the safest region
    # The goal is to turn towards the safest region.
    # Regions_Distances provides a "cost" or "direction" to turn towards front_C.
    # We want to turn towards the safest_region.
    
    # A positive value in Regions_Distances means turning left to get to front_C from that region.
    # A negative value means turning right.
    # So, if safest_region has a positive Regions_Distances value, we need to turn left (positive angular_vel)
    # to align with it. If negative, turn right (negative angular_vel).

    turn_direction_factor = Regions_Distances[safest_region]
    
    if turn_direction_factor > 0: # Safest region is to the left of front_C (or front_C itself)
        Urgency_Report["angular_vel"] = TRANS_ANG_VEL
    elif turn_direction_factor < 0: # Safest region is to the right of front_C
        Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
    else: # safest_region is front_C, but there's still an obstacle, so just rotate to clear
        Urgency_Report["angular_vel"] = TRANS_ANG_VEL # Default to turning left if front_C is safest but still blocked

    # Estimate sleep time based on how much we need to turn to align with the safest region
    # This is a simplification; a more accurate approach would involve PID or continuous adjustment.
    # For now, assume we turn by one regional angle.
    Urgency_Report["sleep"] = abs(REGIONAL_ANGLE * math.pi / 180 / TRANS_ANG_VEL)


def IdentifyRegions(scan):
    """
    TODO: Update Regions_Report using the latest LaserScan.
    - Split the 360° scan into named regions in REGIONS.
    - For each region, record obstacle-relevant range readings.
    END OF TODO
    """
    global Regions_Report
    
    # Clear previous readings
    for region in REGIONS:
        Regions_Report[region] = []

    # Assuming scan.angle_min is -PI, scan.angle_max is PI, and scan.angle_increment is constant
    # The total angle covered by the scan is scan.angle_max - scan.angle_min
    # The number of readings is len(scan.ranges)

    # Calculate the angle covered by each region
    # Total angle is 360 degrees, divided into 12 regions, so 30 degrees per region.
    # Convert to radians: 30 * PI / 180 = PI / 6

    # The regions are defined relative to the robot's front.
    # front_C: -15 to +15 degrees
    # front_L: +15 to +45 degrees
    # left_R: +45 to +75 degrees
    # ... and so on.

    # Angle ranges for each region (in degrees relative to robot's front, 0 is straight ahead)
    # Note: ROS LaserScan angles typically start from the right (negative) and go to the left (positive).
    # So, if 0 is front, positive angles are to the left, negative to the right.

    # Let's define the angle boundaries for each region in radians, relative to the scan's 0 angle (which is usually front).
    # The scan.ranges array is ordered from angle_min to angle_max.
    # We need to map these indices to our defined regions.

    # Assuming scan.angle_min is -PI/2 (right) and scan.angle_max is PI/2 (left) for a 180-degree scan
    # Or -PI to PI for a 360-degree scan. Let's assume 360 for now.

    # Angle ranges in degrees (relative to front, 0 deg)
    # front_C: -15 to 15
    # front_L: 15 to 45
    # left_R: 45 to 75
    # left_C: 75 to 105
    # left_L: 105 to 135
    # back_R: 135 to 165
    # back_C: 165 to 180 and -180 to -165 (split)
    # back_L: -165 to -135
    # right_R: -135 to -105
    # right_C: -105 to -75
    # right_L: -75 to -45
    # front_R: -45 to -15

    num_ranges = len(scan.ranges)
    angle_increment = scan.angle_increment
    min_angle = scan.angle_min

    # Helper to get region index from angle
    def get_region_for_angle(angle_rad):
        angle_deg = math.degrees(angle_rad)
        
        # Normalize angle_deg to be within -180 to 180
        if angle_deg > 180:
            angle_deg -= 360
        elif angle_deg < -180:
            angle_deg += 360

        if -15 <= angle_deg < 15:
            return "front_C"
        elif 15 <= angle_deg < 45:
            return "front_L"
        elif 45 <= angle_deg < 75:
            return "left_R"
        elif 75 <= angle_deg < 105:
            return "left_C"
        elif 105 <= angle_deg < 135:
            return "left_L"
        elif 135 <= angle_deg < 165:
            return "back_R"
        elif 165 <= angle_deg <= 180 or -180 <= angle_deg < -165: # Back_C is split
            return "back_C"
        elif -165 <= angle_deg < -135:
            return "back_L"
        elif -135 <= angle_deg < -105:
            return "right_R"
        elif -105 <= angle_deg < -75:
            return "right_C"
        elif -75 <= angle_deg < -45:
            return "right_L"
        elif -45 <= angle_deg < -15:
            return "front_R"
        else:
            return None # Should not happen if angles cover 360 degrees

    for i in range(num_ranges):
        angle = min_angle + i * angle_increment
        distance = scan.ranges[i]

        # Filter out invalid readings (inf, nan, or too far)
        if not math.isinf(distance) and not math.isnan(distance) and distance > scan.range_min and distance < scan.range_max:
            region_name = get_region_for_angle(angle)
            if region_name:
                Regions_Report[region_name].append(distance)


def Steer(velocity):
    global Urgency_Report

    """
    TODO: Fill a Twist command consistent with Urgency_Report and transition motion.
    - Return the updated Twist.
    END OF TODO
    """
    if Urgency_Report["act"]:
        velocity.linear.x = TRANS_LIN_VEL # Back up slightly
        velocity.angular.z = Urgency_Report["angular_vel"]
    else:
        velocity.linear.x = NORMAL_LIN_VEL
        velocity.angular.z = 0.0
    
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0

    return velocity

class LaserObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("Laser_Obs_Avoid_node")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.vel = Twist()

    def laser_callback(self, msg):
        IdentifyRegions(msg)

    def timer_callback(self):
        global Urgency_Report
        
        ClearanceTest()
        if Urgency_Report["act"]:
            self.vel = Steer(self.vel)
            self.publisher.publish(self.vel)
            # In ROS2, we don't typically use time.sleep() in a callback/timer.
            # The "sleep" duration from Urgency_Report would ideally be handled
            # by a state machine or by adjusting the timer frequency/logic.
            # For this direct port, we'll just publish the command.
            # The effect of "sleep" would be that the robot holds this command
            # for that duration, which is implicitly handled by the timer
            # continuing to publish the same command until ClearanceTest changes Urgency_Report.
        else:
            self.vel.linear.x = NORMAL_LIN_VEL
            self.vel.linear.y = 0.0
            self.vel.linear.z = 0.0
            self.vel.angular.x = 0.0
            self.vel.angular.y = 0.0
            self.vel.angular.z = 0.0
            self.publisher.publish(self.vel)


def main(args=None):
    rclpy.init(args=args)
    node = LaserObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```