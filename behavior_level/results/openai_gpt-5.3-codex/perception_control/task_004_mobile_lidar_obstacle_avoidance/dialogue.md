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

import math
import rclpy
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
    "front_C",
    "front_L",
    "left_R",
    "left_C",
    "left_L",
    "back_R",
    "back_C",
    "back_L",
    "right_R",
    "right_C",
    "right_L",
    "front_R",
]
# this is a global variable that keeps handles the orders for the robot to follow
# if there's a detected object, "act" is turned to True
# and the angular_vel and sleep values are calculated appropriately
Urgency_Report = {"act": False, "angular_vel": 0.0, "sleep": 0.0}
# this dict keeps track of the distance measures for each region
Regions_Report = {
    "front_C": [],
    "front_L": [],
    "left_R": [],
    "left_C": [],
    "left_L": [],
    "back_R": [],
    "back_C": [],
    "back_L": [],
    "right_R": [],
    "right_C": [],
    "right_L": [],
    "front_R": [],
}
# These are the costs to deviate from each region to the goal region (front_C)
Regions_Distances = {
    "front_C": 0,
    "front_L": 1,
    "left_R": 2,
    "left_C": 3,
    "left_L": 4,
    "back_R": 5,
    "back_C": 6,
    "back_L": -5,
    "right_R": -4,
    "right_C": -3,
    "right_L": -2,
    "front_R": -1,
}


# in this function the clearest paths are calculated and the appropriate
# values for the angular_vel and the execution times are assigned
def ClearanceTest():
    """
    TODO: Decide whether to act and update Urgency_Report accordingly.
    - Use Regions_Report / Regions_Distances to choose a safer heading if needed.
    - Update: Urgency_Report["act"], ["angular_vel"], ["sleep"].
    END OF TODO
    """
    global Urgency_Report

    region_min = {}
    for region in REGIONS:
        readings = Regions_Report[region]
        region_min[region] = min(readings) if readings else float("inf")

    forward_blocked = any(
        region_min[r] <= OBSTACLE_DIST for r in ("front_C", "front_L", "front_R")
    )

    if not forward_blocked:
        Urgency_Report["act"] = False
        Urgency_Report["angular_vel"] = 0.0
        Urgency_Report["sleep"] = 0.0
        return

    best_region = max(
        REGIONS,
        key=lambda r: (
            region_min[r] > OBSTACLE_DIST,          # prefer clear sectors
            region_min[r],                          # then largest clearance
            -abs(Regions_Distances[r]),            # then closest to forward
        ),
    )

    steps = Regions_Distances[best_region]
    if steps == 0:
        # if forward was chosen but blocked/noisy, bias to clearer side
        left_score = min(region_min["front_L"], region_min["left_R"], region_min["left_C"])
        right_score = min(region_min["front_R"], region_min["right_L"], region_min["right_C"])
        steps = 1 if left_score >= right_score else -1

    Urgency_Report["act"] = True
    Urgency_Report["angular_vel"] = TRANS_ANG_VEL if steps > 0 else -TRANS_ANG_VEL
    Urgency_Report["sleep"] = abs(steps) * (REGIONAL_ANGLE * PI / 180.0) / TRANS_ANG_VEL


def IdentifyRegions(scan):
    """
    TODO: Update Regions_Report using the latest LaserScan.
    - Split the 360° scan into named regions in REGIONS.
    - For each region, record obstacle-relevant range readings.
    END OF TODO
    """
    for k in REGIONS:
        Regions_Report[k] = []

    sector_rad = math.radians(REGIONAL_ANGLE)
    half_sector = sector_rad / 2.0

    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r):
            continue
        if r < scan.range_min or r > scan.range_max:
            continue

        angle = scan.angle_min + i * scan.angle_increment
        shifted = (angle + half_sector) % (2.0 * math.pi)
        idx = int(shifted / sector_rad) % len(REGIONS)
        Regions_Report[REGIONS[idx]].append(r)


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
        velocity.linear.x = NORMAL_LIN_VEL
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0

    return velocity


def main():
    rclpy.init()
    node = rclpy.create_node("Laser_Obs_Avoid_node")

    # Subscribe to the "/scan" topic in order to read laser scans data from it
    node.create_subscription(LaserScan, "/scan", IdentifyRegions, 10)
    # create our publisher that'll publish to the "/cmd_vel" topic
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    vel = Twist()
    # ros will try to run this code 10 times/second
    rate = node.create_rate(10)  # 10Hz

    try:
        # keep running while ROS2 is not shutdown
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            # Need a do{ ... }while(); here (C is awesome)
            # Since I need to check at least once the clearance
            done = False
            while not done and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                ClearanceTest()
                if Urgency_Report["act"]:
                    vel = Steer(vel)
                    pub.publish(vel)
                else:
                    done = True
            # This else belongs to the while(), and the code below it could be cleaned furthermore
            else:
                vel.linear.x = NORMAL_LIN_VEL
                vel.linear.y = 0.0
                vel.linear.z = 0.0
                vel.angular.x = 0.0
                vel.angular.y = 0.0
                vel.angular.z = 0.0
                pub.publish(vel)

            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
