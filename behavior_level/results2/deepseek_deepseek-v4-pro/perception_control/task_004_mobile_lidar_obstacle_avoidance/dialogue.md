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
----------------------------
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


def ClearanceTest():
    global Urgency_Report, Regions_Report, Regions_Distances

    Urgency_Report["act"] = False
    Urgency_Report["angular_vel"] = 0.0
    Urgency_Report["sleep"] = 0

    obstacle_regions = []
    for region in REGIONS:
        distances = Regions_Report[region]
        if distances:
            min_dist = min(distances)
            if min_dist < OBSTACLE_DIST:
                obstacle_regions.append(region)

    if not obstacle_regions:
        return

    Urgency_Report["act"] = True

    best_region = None
    best_cost = float('inf')
    for region in REGIONS:
        if region not in obstacle_regions:
            cost = abs(Regions_Distances[region])
            if cost < best_cost:
                best_cost = cost
                best_region = region

    if best_region is None:
        Urgency_Report["angular_vel"] = TRANS_ANG_VEL
        Urgency_Report["sleep"] = 1
        return

    region_distance = Regions_Distances[best_region]
    if region_distance > 0:
        Urgency_Report["angular_vel"] = TRANS_ANG_VEL
    elif region_distance < 0:
        Urgency_Report["angular_vel"] = -TRANS_ANG_VEL
    else:
        Urgency_Report["angular_vel"] = 0.0

    Urgency_Report["sleep"] = int(abs(region_distance) * 0.5) + 1


def IdentifyRegions(scan):
    global Regions_Report

    for region in REGIONS:
        Regions_Report[region] = []

    if len(scan.ranges) == 0:
        return

    angle_min = scan.angle_min
    angle_increment = scan.angle_increment
    range_max = scan.range_max

    region_angle_rad = REGIONAL_ANGLE * PI / 180.0

    for region in REGIONS:
        if region == "front_C":
            start_angle = -region_angle_rad / 2.0
            end_angle = region_angle_rad / 2.0
        elif region == "front_L":
            start_angle = region_angle_rad / 2.0
            end_angle = 3.0 * region_angle_rad / 2.0
        elif region == "left_R":
            start_angle = 3.0 * region_angle_rad / 2.0
            end_angle = 5.0 * region_angle_rad / 2.0
        elif region == "left_C":
            start_angle = 5.0 * region_angle_rad / 2.0
            end_angle = 7.0 * region_angle_rad / 2.0
        elif region == "left_L":
            start_angle = 7.0 * region_angle_rad / 2.0
            end_angle = 9.0 * region_angle_rad / 2.0
        elif region == "back_R":
            start_angle = 9.0 * region_angle_rad / 2.0
            end_angle = 11.0 * region_angle_rad / 2.0
        elif region == "back_C":
            start_angle = 11.0 * region_angle_rad / 2.0
            end_angle = -11.0 * region_angle_rad / 2.0
        elif region == "back_L":
            start_angle = -11.0 * region_angle_rad / 2.0
            end_angle = -9.0 * region_angle_rad / 2.0
        elif region == "right_R":
            start_angle = -9.0 * region_angle_rad / 2.0
            end_angle = -7.0 * region_angle_rad / 2.0
        elif region == "right_C":
            start_angle = -7.0 * region_angle_rad / 2.0
            end_angle = -5.0 * region_angle_rad / 2.0
        elif region == "right_L":
            start_angle = -5.0 * region_angle_rad / 2.0
            end_angle = -3.0 * region_angle_rad / 2.0
        elif region == "front_R":
            start_angle = -3.0 * region_angle_rad / 2.0
            end_angle = -region_angle_rad / 2.0
        else:
            continue

        start_index = int((start_angle - angle_min) / angle_increment)
        end_index = int((end_angle - angle_min) / angle_increment)

        if start_index < 0:
            start_index += len(scan.ranges)
        if end_index < 0:
            end_index += len(scan.ranges)

        if start_index <= end_index:
            indices = range(start_index, min(end_index + 1, len(scan.ranges)))
        else:
            indices = list(range(start_index, len(scan.ranges))) + list(range(0, min(end_index + 1, len(scan.ranges))))

        for i in indices:
            if 0 <= i < len(scan.ranges):
                r = scan.ranges[i]
                if scan.range_min <= r <= range_max:
                    Regions_Report[region].append(r)


def Steer(velocity):
    global Urgency_Report

    velocity.linear.x = TRANS_LIN_VEL
    velocity.linear.y = 0.0
    velocity.linear.z = 0.0
    velocity.angular.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z = Urgency_Report["angular_vel"]

    return velocity


class LaserObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__("Laser_Obs_Avoid_node")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vel = Twist()

    def scan_callback(self, msg):
        IdentifyRegions(msg)

    def timer_callback(self):
        done = False
        while not done:
            ClearanceTest()
            if Urgency_Report["act"]:
                self.vel = Steer(self.vel)
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
    node = LaserObstacleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
----------------------------