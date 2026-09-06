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
                     "front_C":[], "front_L":[], "left_R":[],
                     "left_C":[], "left_L":[], "back_R":[],
                     "back_C":[], "back_L":[], "right_R":[],
                     "right_C":[], "right_L":[], "front_R":[],
                 }

Regions_Distances = {
                     "front_C": 0, "front_L": 1, "left_R": 2,
                     "left_C": 3, "left_L": 4, "back_R": 5,
                     "back_C": 6, "back_L": -5, "right_R": -4,
                     "right_C": -3, "right_L": -2, "front_R": -1,
                 }


def ClearanceTest():
    global Urgency_Report

    # Check if any front-facing regions have obstacles
    front_regions = ["front_C", "front_L", "front_R"]
    obstacle_detected = False

    for region in front_regions:
        readings = Regions_Report.get(region, [])
        if len(readings) > 0:
            min_dist = min(readings)
            if min_dist < OBSTACLE_DIST:
                obstacle_detected = True
                break

    if not obstacle_detected:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0
        return

    # Find the clearest region (highest minimum distance)
    best_region = None
    best_min_dist = -1.0

    for region in REGIONS:
        readings = Regions_Report.get(region, [])
        if len(readings) > 0:
            min_dist = min(readings)
            if min_dist > best_min_dist:
                best_min_dist = min_dist
                best_region = region

    if best_region is None:
        best_region = "back_C"

    # Determine angular velocity direction and magnitude based on region cost
    cost = Regions_Distances.get(best_region, 0)

    if cost == 0:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0
    else:
        Urgency_Report["act"] = True
        if cost > 0:
            Urgency_Report["angular_vel"] = TRANS_ANG_VEL
        else:
            Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
        Urgency_Report["sleep"] = abs(cost)


def IdentifyRegions(scan):
    global Regions_Report

    total_readings = len(scan.ranges)
    if total_readings == 0:
        return

    readings_per_region = total_readings // len(REGIONS)

    for i, region in enumerate(REGIONS):
        start_index = i * readings_per_region
        end_index = start_index + readings_per_region
        if i == len(REGIONS) - 1:
            end_index = total_readings

        region_readings = []
        for j in range(start_index, end_index):
            r = scan.ranges[j]
            if scan.range_min <= r <= scan.range_max:
                region_readings.append(r)

        if len(region_readings) == 0:
            region_readings = [scan.range_max]

        Regions_Report[region] = region_readings


def Steer(velocity):
    global Urgency_Report

    velocity.linear.x = TRANS_LIN_VEL
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = Urgency_Report["angular_vel"]

    return velocity


class LaserObsAvoidNode(Node):
    def __init__(self):
        super().__init__('Laser_Obs_Avoid_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            IdentifyRegions,
            10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        vel = Twist()

        done = False
        while not done:
            ClearanceTest()
            if Urgency_Report["act"]:
                vel = Steer(vel)
                self.pub.publish(vel)
            else:
                done = True
        else:
            vel.linear.x = NORMAL_LIN_VEL
            vel.linear.y = 0.0
            vel.linear.z = 0.0
            vel.angular.x = 0.0
            vel.angular.y = 0.0
            vel.angular.z = 0.0
            self.pub.publish(vel)


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