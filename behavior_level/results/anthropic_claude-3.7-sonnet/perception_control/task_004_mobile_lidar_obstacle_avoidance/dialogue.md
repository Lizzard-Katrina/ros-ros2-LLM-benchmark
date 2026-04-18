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

# obstacle threshhold, objects a this distance or below it
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


def ClearanceTest():
    """
    Decide whether to act and update Urgency_Report accordingly.
    """
    global Urgency_Report, Regions_Report, Regions_Distances
    
    # Reset urgency report
    Urgency_Report["act"] = False
    Urgency_Report["angular_vel"] = 0.0
    Urgency_Report["sleep"] = 0
    
    # Check if any region has obstacles
    has_obstacle = False
    for region in REGIONS:
        if Regions_Report[region] and min(Regions_Report[region]) <= OBSTACLE_DIST:
            has_obstacle = True
            break
    
    if not has_obstacle:
        return
    
    # Find the safest region (furthest from obstacles)
    safest_region = None
    max_clearance = 0
    
    for region in REGIONS:
        if not Regions_Report[region]:
            continue
        
        min_distance = min(Regions_Report[region])
        if min_distance > max_clearance:
            max_clearance = min_distance
            safest_region = region
    
    if safest_region and safest_region != "front_C":
        Urgency_Report["act"] = True
        # Calculate angular velocity based on region distance from front_C
        Urgency_Report["angular_vel"] = TRANS_ANG_VEL * (1 if Regions_Distances[safest_region] > 0 else -1)
        # Set sleep time proportional to the angular distance
        Urgency_Report["sleep"] = abs(Regions_Distances[safest_region]) * 0.1


def IdentifyRegions(scan):
    """
    Update Regions_Report using the latest LaserScan.
    """
    global Regions_Report
    
    # Clear previous readings
    for region in REGIONS:
        Regions_Report[region] = []
    
    # Get scan parameters
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment
    ranges = scan.ranges
    
    # Process each reading
    for i, range_val in enumerate(ranges):
        # Calculate the angle of this reading
        angle = angle_min + i * angle_increment
        
        # Normalize angle to [0, 2π)
        while angle < 0:
            angle += 2 * PI
        while angle >= 2 * PI:
            angle -= 2 * PI
        
        # Determine which region this reading belongs to
        region_index = int((angle * 180 / PI) // REGIONAL_ANGLE)
        if region_index >= len(REGIONS):
            region_index = 0
            
        region = REGIONS[region_index]
        
        # Add valid range readings to the appropriate region
        if not (range_val < scan.range_min or range_val > scan.range_max):
            Regions_Report[region].append(range_val)


def Steer(velocity):
    global Urgency_Report
    
    # Create a new Twist message
    vel = Twist()
    
    if Urgency_Report["act"]:
        # Set linear velocity for transition
        vel.linear.x = TRANS_LIN_VEL
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        
        # Set angular velocity based on urgency report
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = Urgency_Report["angular_vel"]
    else:
        # Normal forward motion
        vel.linear.x = NORMAL_LIN_VEL
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
    
    return vel


class LaserObsAvoidNode(Node):
    def __init__(self):
        super().__init__("laser_obs_avoid_node")
        
        # Subscribe to the "/scan" topic
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            IdentifyRegions,
            10
        )
        
        # Create publisher for "/cmd_vel"
        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.vel = Twist()
    
    def control_loop(self):
        # Need a do{ ... }while(); here
        done = False
        while not done:
            ClearanceTest()
            if Urgency_Report["act"]:
                self.vel = Steer(self.vel)
                self.publisher.publish(self.vel)
            else:
                done = True
        # This else belongs to the while()
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