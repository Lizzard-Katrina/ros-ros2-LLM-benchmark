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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

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
    "front_C": [], "front_L": [], "left_R": [],
    "left_C": [], "left_L": [], "back_R": [],
    "back_C": [], "back_L": [], "right_R": [],
    "right_C": [], "right_L": [], "front_R": [],
}

Regions_Distances = {
    "front_C": 0, "front_L": 1, "left_R": 2,
    "left_C": 3, "left_L": 4, "back_R": 5,
    "back_C": 6, "back_L": -5, "right_R": -4,
    "right_C": -3, "right_L": -2, "front_R": -1,
}

class LaserObsAvoidNode(Node):
    def __init__(self):
        super().__init__("Laser_Obs_Avoid_node")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.IdentifyRegions,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            1
        )
        self.urgency_report = Urgency_Report.copy()
        self.regions_report = Regions_Report.copy()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vel = Twist()

    def ClearanceTest(self):
        min_distance = float('inf')
        safest_region = "front_C"
        
        for region in REGIONS:
            if self.regions_report[region]:
                region_min = min(self.regions_report[region])
                if region_min < min_distance:
                    min_distance = region_min
                    safest_region = region
        
        if min_distance < OBSTACLE_DIST:
            self.urgency_report["act"] = True
            angle_diff = Regions_Distances[safest_region] * (REGIONAL_ANGLE * PI / 180)
            self.urgency_report["angular_vel"] = TRANS_ANG_VEL * (angle_diff / abs(angle_diff) if angle_diff != 0 else 0)
            self.urgency_report["sleep"] = int(abs(angle_diff) / TRANS_ANG_VEL * 10)
        else:
            self.urgency_report["act"] = False
            self.urgency_report["angular_vel"] = 0.0
            self.urgency_report["sleep"] = 0

    def IdentifyRegions(self, scan):
        num_readings = len(scan.ranges)
        angle_increment = scan.angle_increment
        
        for region in REGIONS:
            self.regions_report[region] = []
        
        for i in range(num_readings):
            angle = scan.angle_min + i * angle_increment
            distance = scan.ranges[i]
            
            if not (scan.range_min < distance < scan.range_max):
                continue
            
            angle_deg = angle * 180 / PI
            normalized_angle = (angle_deg + 360) % 360
            
            if 0 <= normalized_angle < 30 or 330 <= normalized_angle < 360:
                self.regions_report["front_C"].append(distance)
            elif 30 <= normalized_angle < 60:
                self.regions_report["front_L"].append(distance)
            elif 60 <= normalized_angle < 90:
                self.regions_report["left_R"].append(distance)
            elif 90 <= normalized_angle < 120:
                self.regions_report["left_C"].append(distance)
            elif 120 <= normalized_angle < 150:
                self.regions_report["left_L"].append(distance)
            elif 150 <= normalized_angle < 180:
                self.regions_report["back_R"].append(distance)
            elif 180 <= normalized_angle < 210:
                self.regions_report["back_C"].append(distance)
            elif 210 <= normalized_angle < 240:
                self.regions_report["back_L"].append(distance)
            elif 240 <= normalized_angle < 270:
                self.regions_report["right_R"].append(distance)
            elif 270 <= normalized_angle < 300:
                self.regions_report["right_C"].append(distance)
            elif 300 <= normalized_angle < 330:
                self.regions_report["right_L"].append(distance)
            elif 330 <= normalized_angle < 360:
                self.regions_report["front_R"].append(distance)

    def Steer(self, velocity):
        if self.urgency_report["act"]:
            velocity.linear.x = TRANS_LIN_VEL
            velocity.angular.z = self.urgency_report["angular_vel"]
        else:
            velocity.linear.x = NORMAL_LIN_VEL
            velocity.angular.z = 0.0
        
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        
        return velocity

    def timer_callback(self):
        done = False
        while not done:
            self.ClearanceTest()
            if self.urgency_report["act"]:
                self.vel = self.Steer(self.vel)
                self.publisher.publish(self.vel)
            else:
                done = True
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
```