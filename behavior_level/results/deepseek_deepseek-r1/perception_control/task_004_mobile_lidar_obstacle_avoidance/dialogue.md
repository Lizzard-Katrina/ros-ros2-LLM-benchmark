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

class LaserObsAvoidNode(Node):
    def __init__(self):
        super().__init__("Laser_Obs_Avoid_node")
        self.Urgency_Report = {
            "act": False, "angular_vel": 0.0, "sleep": 0
        }
        self.Regions_Report = {
            "front_C": [], "front_L": [], "left_R": [],
            "left_C": [], "left_L": [], "back_R": [],
            "back_C": [], "back_L": [], "right_R": [],
            "right_C": [], "right_L": [], "front_R": [],
        }
        self.Regions_Distances = {
            "front_C": 0, "front_L": 1, "left_R": 2,
            "left_C": 3, "left_L": 4, "back_R": 5,
            "back_C": 6, "back_L": -5, "right_R": -4,
            "right_C": -3, "right_L": -2, "front_R": -1,
        }
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.IdentifyRegions,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vel = Twist()

    def ClearanceTest(self):
        min_cost = float('inf')
        best_region = "front_C"
        
        for region in REGIONS:
            if not self.Regions_Report[region]:
                continue
                
            min_dist = min(self.Regions_Report[region])
            if min_dist > OBSTACLE_DIST:
                cost = abs(self.Regions_Distances[region])
                if cost < min_cost:
                    min_cost = cost
                    best_region = region
        
        if best_region != "front_C":
            self.Urgency_Report["act"] = True
            self.Urgency_Report["angular_vel"] = self.Regions_Distances[best_region] * 0.5
            self.Urgency_Report["sleep"] = 1
        else:
            self.Urgency_Report["act"] = False
            self.Urgency_Report["angular_vel"] = 0.0
            self.Urgency_Report["sleep"] = 0

    def IdentifyRegions(self, scan):
        angle_increment = scan.angle_increment
        num_readings = len(scan.ranges)
        
        for i in range(num_readings):
            angle = scan.angle_min + i * angle_increment
            angle_deg = angle * 180.0 / PI
            
            if -15 <= angle_deg < 15:
                self.Regions_Report["front_C"].append(scan.ranges[i])
            elif 15 <= angle_deg < 45:
                self.Regions_Report["front_L"].append(scan.ranges[i])
            elif 45 <= angle_deg < 75:
                self.Regions_Report["left_R"].append(scan.ranges[i])
            elif 75 <= angle_deg < 105:
                self.Regions_Report["left_C"].append(scan.ranges[i])
            elif 105 <= angle_deg < 135:
                self.Regions_Report["left_L"].append(scan.ranges[i])
            elif 135 <= angle_deg < 165:
                self.Regions_Report["back_R"].append(scan.ranges[i])
            elif 165 <= angle_deg <= 180 or -180 <= angle_deg < -165:
                self.Regions_Report["back_C"].append(scan.ranges[i])
            elif -165 <= angle_deg < -135:
                self.Regions_Report["back_L"].append(scan.ranges[i])
            elif -135 <= angle_deg < -105:
                self.Regions_Report["right_R"].append(scan.ranges[i])
            elif -105 <= angle_deg < -75:
                self.Regions_Report["right_C"].append(scan.ranges[i])
            elif -75 <= angle_deg < -45:
                self.Regions_Report["right_L"].append(scan.ranges[i])
            elif -45 <= angle_deg < -15:
                self.Regions_Report["front_R"].append(scan.ranges[i])
        
        for region in REGIONS:
            self.Regions_Report[region] = [r for r in self.Regions_Report[region] if not math.isinf(r)]

    def Steer(self, velocity):
        velocity.linear.x = TRANS_LIN_VEL
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = self.Urgency_Report["angular_vel"] * TRANS_ANG_VEL
        return velocity

    def timer_callback(self):
        done = False
        while not done:
            self.ClearanceTest()
            if self.Urgency_Report["act"]:
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()