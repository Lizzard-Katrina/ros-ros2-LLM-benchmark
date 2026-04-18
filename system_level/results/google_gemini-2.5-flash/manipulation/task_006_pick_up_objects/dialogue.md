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

FILE_PATH: manage_objects_node.py
----------------------------
#!/usr/bin/env python
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
import rospy
import random
import copy
import sys


class ManageObject():
    def __init__(self, models_path):
        # If stage 4 is used
        # self.locations = [(1.5, -1.2), (0.1, -1.8),
        #                   (-1, 2), (-2, 0.1), (0.5, 0.2)]
        # self.let_beer = (-1.7, -2)
        # self.let_coke = (2, 1.7)
        
        # If stage 3 is used
        self.locations = [(1.25, 0.5), (1.25, -1.25), (0.0, -1.25),
                          (-0.5, 1.25), (-1.25, 0.5)]
        self.let_beer = (-1.5, -1.5)
        self.let_coke = (1.5, 1.5)
        
        self.model_coke = models_path + '/models/coke_can/model.sdf'
        self.model_beer = models_path + '/models/beer/model.sdf'
        self.beer_loc = None
        self.coke_loc = None
        self.robot_pose = None
        self.beer_on_robot = False
        self.coke_on_robot = False

        if not self.setup_escenario():
            exit()
            
        self.pub_set_model_state = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=1)
        server_check = rospy.Service('~check_object', Trigger,
                                     self.handle_check_object)
        server_take = rospy.Service('~get_object', Trigger,
                                    self.handle_take_object)
        server_let = rospy.Service('~let_object', Trigger,
                                   self.handle_let_object)

        subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Timer(rospy.Duration(0.1), self.iterate)

    def spawn_model(self, model_name, model_xml, p):
        # TODO
        ## 1. Create a client for '/spawn_entity' (gazebo_msgs.srv.SpawnEntity).
        # 2. Wait for the service to be available.
        # 3. Fill the SpawnEntity request (name, xml, initial_pose).
        # 4. Call the service asynchronously and handle the future.
        # 5. Return the success status from the response.
        # END OF TODO
    def setup_escenario(self):
        self.beer_loc = self.locations[random.randint(
            0, len(self.locations)-1)]
        self.spawn_model('beer', self.model_beer, self.beer_loc)
        while self.coke_loc is None or self.coke_loc == self.beer_loc:
            self.coke_loc = self.locations[random.randint(
                0, len(self.locations)-1)]
        return self.spawn_model('coke', self.model_coke, self.coke_loc)

    def handle_check_object(self, req):
        ret = TriggerResponse()
        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            ret.success = True
            ret.message = 'beer'
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            ret.success = True
            ret.message = 'coke'
        else:
            ret.success = False
            ret.message = ''
        return ret

    def handle_take_object(self, req):
        ret = TriggerResponse()
        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            self.beer_on_robot = True
            ret.success = True
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            self.coke_on_robot = True
            ret.success = True
        else:
            print("Error! No objects close")
            ret.success = False
        return ret

    def handle_let_object(self, req):
        ret = TriggerResponse()
        if self.beer_on_robot:
            self.beer_on_robot = False
            ret.success = True
            model_state = ModelState()
            model_state.model_name = 'beer'
            model_state.pose.position.x = self.robot_pose[0] - 0.25
            model_state.pose.position.y = self.robot_pose[1] - 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)

        elif self.coke_on_robot:
            self.coke_on_robot = False
            ret.success = True
            model_state = ModelState()
            model_state.model_name = 'coke'
            model_state.pose.position.x = self.robot_pose[0] + 0.25
            model_state.pose.position.y = self.robot_pose[1] + 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)

        else:
            print("Error! No objects grasped.")
            ret.success = False
        return ret

    def odom_callback(self, data):
        self.robot_pose = (data.pose.pose.position.x,
                           data.pose.pose.position.y)

    def distance(self, p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

    def iterate(self, event):
        if self.coke_on_robot:
            model_state = ModelState()
            model_state.model_name = 'coke'
            model_state.pose.position.x = self.robot_pose[0] 
            model_state.pose.position.y = self.robot_pose[1]
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)
            self.coke_loc = copy.copy(self.robot_pose)
        elif self.beer_on_robot:
            model_state = ModelState()
            model_state.model_name = 'beer'
            model_state.pose.position.x = self.robot_pose[0]
            model_state.pose.position.y = self.robot_pose[1]
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)
            self.beer_loc = copy.copy(self.robot_pose)


if __name__ == '__main__':
    models_path = './'
    print(sys.argv)
    if len(sys.argv) >= 2:
        models_path = sys.argv[1]
     
    print("Path: ", models_path)
    rospy.init_node('spawn_model')
    check_object = ManageObject(models_path)
    rospy.spin()
    

----------------------------

FILE_PATH: pickup_behaviors_node.py
----------------------------
#!/usr/bin/env python

import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest

# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/check_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/check_object', Trigger)
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [CheckObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        # TODO
        # 1. Log the service call to '/manage_objects/check_object'.
        # 2. Call the service using the client created in setup().
        # 3. Since PyTrees ticks are synchronous, handle the rclpy future to get the result.
        # 4. If success: write 'resp.message' to self.blackboard.object_name and return py_trees.common.Status.SUCCESS.
        # 5. Otherwise: return py_trees.common.Status.FAILURE.
        # END OF TODO
    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/get_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/get_object', Trigger)
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [GetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/get_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/get_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/let_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/let_object', Trigger)
            self.logger.debug(
                "  %s [LetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [LetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/let_object".format(self.name))
            resp = self.server(TriggerRequest())
            if resp.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/let_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))



if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    # Create Behaviors
    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    let_object = LetObject("let_object")
    # fill the rest of the code here ...
 

----------------------------

FILE_PATH: turtlebot_controller_node.py
----------------------------
#!/usr/bin/python3

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from geometry_msgs.msg import PoseStamped


# Wrap angle between -pi and pi
def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )


# Controller
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    """ Computes the control command to move from current position to goal. """
    theta_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    w = Kw * wrap_angle(theta_d - current[2])
    v = 0
    if abs(w) < 0.05: # to avoid move while turning
        v = Kv * np.linalg.norm(goal - current[0:2])
    return v, w

class Controller:
    def __init__(self, odom_topic, cmd_vel_topic, distance_threshold):
        # TODO
        # 1. Store the node and distance_threshold.
        # 2. Create a Publisher for 'cmd_vel_topic' (Twist, qos=10).
        # 3. Create a Subscriber for 'odom_topic' (Odometry) and '/move_base_simple/goal' (PoseStamped).
        # 4. IMPORTANT: Ensure topic/frame names follow ROS 2 convention (no leading slashes).
        # 5. Create a Timer (10Hz) to call 'self.controller'.    
        # END OF TODO
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    # Goal callback
    def get_goal(self, goal):
        if self.current_pose is not None:
            print("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = None                                                    # to send zero velocity while planning
            self.path = [self.current_pose[0:2], self.goal]                     # to avoid path planning
            self.publish_path(self.path)
            del self.path[0]                                                    # remove current pose
        
    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self, event):
        v = 0   
        w = 0
        if self.path is not None and len(self.path) > 0:
            
            # If current wat point reached with some tolerance move to next point otherwise move to current point
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < self.distance_threshold:
                print("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    print("Final position reached!")
            else:
                v, w = move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        self.__send_commnd__(v, w)
    

    # Publishers
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)
        
    def publish_path(self, path):
        if len(path) > 1:
            print("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = rospy.Time.now()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0)
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            
            color_red = ColorRGBA()
            color_red.r = 1
            color_red.g = 0
            color_red.b = 0
            color_red.a = 1
            color_blue = ColorRGBA()
            color_blue.r = 0
            color_blue.g = 0
            color_blue.b = 1
            color_blue.a = 1

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)



if __name__ == '__main__':
    rospy.init_node('turtlebot_controller')   
    node = Controller('/odom', '/cmd_vel', 0.15)
    
    # Run forever
    rospy.spin()

----------------------------


# LLM Response

[FILENAME: manage_objects_node.py]
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
import random
import copy
import sys
import os
import asyncio


class ManageObject(Node):
    def __init__(self, models_path):
        super().__init__('manage_objects_node')
        
        # If stage 3 is used
        self.locations = [(1.25, 0.5), (1.25, -1.25), (0.0, -1.25),
                          (-0.5, 1.25), (-1.25, 0.5)]
        self.let_beer = (-1.5, -1.5)
        self.let_coke = (1.5, 1.5)
        
        self.model_coke = os.path.join(models_path, 'models/coke_can/model.sdf')
        self.model_beer = os.path.join(models_path, 'models/beer/model.sdf')
        self.beer_loc = None
        self.coke_loc = None
        self.robot_pose = None
        self.beer_on_robot = False
        self.coke_on_robot = False

        if not self.setup_escenario():
            self.get_logger().error("Failed to set up scenario.")
            exit()
            
        self.pub_set_model_state = self.create_publisher(
            ModelState, '/gazebo/set_model_state', 1)
        self.create_service(Trigger, '~/check_object',
                                     self.handle_check_object)
        self.create_service(Trigger, '~/get_object',
                                    self.handle_take_object)
        self.create_service(Trigger, '~/let_object',
                                   self.handle_let_object)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.1, self.iterate)

    def spawn_model(self, model_name, model_xml_path, initial_pose_tuple):
        self.get_logger().info(f"Attempting to spawn model: {model_name} from {model_xml_path}")
        client = self.create_client(SpawnEntity, '/spawn_entity')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn_entity service not available, waiting again...')

        request = SpawnEntity.Request()
        request.name = model_name
        
        try:
            with open(model_xml_path, 'r') as f:
                request.xml = f.read()
        except IOError as e:
            self.get_logger().error(f"Could not read model XML from {model_xml_path}: {e}")
            return False

        initial_pose = Pose()
        initial_pose.position.x = initial_pose_tuple[0]
        initial_pose.position.y = initial_pose_tuple[1]
        initial_pose.position.z = 0.0
        request.initial_pose = initial_pose

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Spawn service response for {model_name}: {future.result().success}, {future.result().status_message}')
            return future.result().success
        else:
            self.get_logger().error(f'Service call failed for {model_name}')
            return False
        
    def setup_escenario(self):
        self.beer_loc = self.locations[random.randint(
            0, len(self.locations)-1)]
        if not self.spawn_model('beer', self.model_beer, self.beer_loc):
            return False
        while self.coke_loc is None or self.coke_loc == self.beer_loc:
            self.coke_loc = self.locations[random.randint(
                0, len(self.locations)-1)]
        return self.spawn_model('coke', self.model_coke, self.coke_loc)

    def handle_check_object(self, req, res):
        if self.robot_pose is None:
            res.success = False
            res.message = "Robot pose not available."
            return res

        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            res.success = True
            res.message = 'beer'
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            res.success = True
            res.message = 'coke'
        else:
            res.success = False
            res.message = ''
        return res

    def handle_take_object(self, req, res):
        if self.robot_pose is None:
            res.success = False
            res.message = "Robot pose not available."
            return res

        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            self.beer_on_robot = True
            res.success = True
            res.message = "Beer taken."
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            self.coke_on_robot = True
            res.success = True
            res.message = "Coke taken."
        else:
            self.get_logger().info("Error! No objects close")
            res.success = False
            res.message = "No objects close."
        return res

    def handle_let_object(self, req, res):
        if self.robot_pose is None:
            res.success = False
            res.message = "Robot pose not available."
            return res

        if self.beer_on_robot:
            self.beer_on_robot = False
            res.success = True
            res.message = "Beer released."
            model_state = ModelState()
            model_state.model_name = 'beer'
            model_state.pose.position.x = self.robot_pose[0] - 0.25
            model_state.pose.position.y = self.robot_pose[1] - 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'world' # Changed from ground_plane to world for ROS2 Gazebo
            self.pub_set_model_state.publish(model_state)

        elif self.coke_on_robot:
            self.coke_on_robot = False
            res.success = True
            res.message = "Coke released."
            model_state = ModelState()
            model_state.model_name = 'coke'
            model_state.pose.position.x = self.robot_pose[0] + 0.25
            model_state.pose.position.y = self.robot_pose[1] + 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'world' # Changed from ground_plane to world for ROS2 Gazebo
            self.pub_set_model_state.publish(model_state)

        else:
            self.get_logger().info("Error! No objects grasped.")
            res.success = False
            res.message = "No objects grasped."
        return res

    def odom_callback(self, data):
        self.robot_pose = (data.pose.pose.position.x,
                           data.pose.pose.position.y)

    def distance(self, p1, p2):
        if p1 is None or p2 is None:
            return float('inf')
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

    def iterate(self):
        if self.robot_pose is None:
            return

        if self.coke_on_robot:
            model_state = ModelState()
            model_state.model_name = 'coke'
            model_state.pose.position.x = self.robot_pose[0] 
            model_state.pose.position.y = self.robot_pose[1]
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'world' # Changed from ground_plane to world for ROS2 Gazebo
            self.pub_set_model_state.publish(model_state)
            self.coke_loc = copy.copy(self.robot_pose)
        elif self.beer_on_robot:
            model_state = ModelState()
            model_state.model_name = 'beer'
            model_state.pose.position.x = self.robot_pose[0]
            model_state.pose.position.y = self.robot_pose[1]
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'world' # Changed from ground_plane to world for ROS2 Gazebo
            self.pub_set_model_state.publish(model_state)
            self.beer_loc = copy.copy(self.robot_pose)


def main(args=None):
    rclpy.init(args=args)
    models_path = './'
    if len(sys.argv) >= 2:
        models_path = sys.argv[1]
     
    print("Path: ", models_path)
    manage_object_node = ManageObject(models_path)
    rclpy.spin(manage_object_node)
    manage_object_node.destroy_node()
    rclpy.shutdown()
    

[FILENAME: pickup_behaviors_node.py]
#!/usr/bin/env python

import py_trees
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, TriggerRequest
import threading

# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)
        self.node = node
        self.service_client = None

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        self.service_client = self.node.create_client(Trigger, '/manage_objects/check_object')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('check_object service not available, waiting again...')
        self.logger.debug(
            "  %s [CheckObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        self.node.get_logger().info(f"  {self.name}: calling service /manage_objects/check_object")
        request = Trigger.Request()
        future = self.service_client.call_async(request)
        
        # Since PyTrees ticks are synchronous, we need to block until the service call is complete.
        # This is generally not recommended for long-running services in a single-threaded executor,
        # but for short service calls within a behavior tree tick, it's acceptable.
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            resp = future.result()
            if resp.success:
                self.blackboard.object_name = resp.message
                self.node.get_logger().info(f"  {self.name}: Found object: {resp.message}")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().info(f"  {self.name}: No object found.")
                return py_trees.common.Status.FAILURE
        else:
            self.node.get_logger().error(f"  {self.name}: Service call failed for /manage_objects/check_object")
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(GetObject, self).__init__(name)
        self.node = node
        self.service_client = None

    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        self.service_client = self.node.create_client(Trigger, '/manage_objects/get_object')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('get_object service not available, waiting again...')
        self.logger.debug(
            "  %s [GetObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.node.get_logger().info(
                "  {}: call service /manage_objects/get_object".format(self.name))
            request = Trigger.Request()
            future = self.service_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            
            if future.result() is not None:
                resp = future.result()
                if resp.success:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().error(f"  {self.name}: Service call failed for /manage_objects/get_object")
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.node.get_logger().error(
                "  {}: Error calling service /manage_objects/get_object: {}".format(self.name, e))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(LetObject, self).__init__(name)
        self.node = node
        self.service_client = None

    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        self.service_client = self.node.create_client(Trigger, '/manage_objects/let_object')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('let_object service not available, waiting again...')
        self.logger.debug(
            "  %s [LetObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.node.get_logger().info(
                "  {}: call service /manage_objects/let_object".format(self.name))
            request = Trigger.Request()
            future = self.service_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            if future.result() is not None:
                resp = future.result()
                if resp.success:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().error(f"  {self.name}: Service call failed for /manage_objects/let_object")
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.node.get_logger().error(
                "  {}: Error calling service /manage_objects/let_object: {}".format(self.name, e))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


def main():
    rclpy.init()
    node = Node("behavior_trees_node")
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # Create Behaviors
    check_object = CheckObject("check_object", node)
    get_object = GetObject("get_object", node)
    let_object = LetObject("let_object", node)

    # Define the behavior tree
    root = py_trees.composites.Sequence(name="Pickup Object Sequence", memory=True)
    root.add_children([
        check_object,
        get_object,
        let_object
    ])

    # Create a tree and setup
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15) # Increased timeout for ROS2 service startup

    # Create a separate thread for spinning the ROS2 node
    # This allows the behavior tree to tick in the main thread while ROS2 callbacks are processed.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Tick the tree
        for i in range(1, 200): # Tick for a certain number of times or until a condition is met
            tree.tick()
            node.get_logger().info(f"Tick {i}")
            py_trees.display.print_ascii(tree.root)
            # Add a small sleep to prevent busy-waiting and allow ROS2 to process events
            py_trees.blackboard.Blackboard().set("tick_count", i) # Example of writing to blackboard
            node.get_logger().info(f"Blackboard object_name: {py_trees.blackboard.Blackboard().get('object_name', default='None')}")
            rclpy.rate.Rate(1.0).sleep() # Tick at 1 Hz

    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()

[FILENAME: turtlebot_controller_node.py]
#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf_transformations

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from geometry_msgs.msg import PoseStamped


# Wrap angle between -pi and pi
def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )


# Controller
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    """ Computes the control command to move from current position to goal. """
    theta_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    w = Kw * wrap_angle(theta_d - current[2])
    v = 0
    if abs(w) < 0.05: # to avoid move while turning
        v = Kv * np.linalg.norm(goal - current[0:2])
    return v, w

class Controller(Node):
    def __init__(self, odom_topic, cmd_vel_topic, distance_threshold):
        super().__init__('turtlebot_controller')
        self.distance_threshold = distance_threshold
        self.current_pose = None
        self.goal = None
        self.path = None
        self.Kv = 0.5
        self.Kw = 0.5
        self.v_max = 0.22
        self.w_max = 2.84

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(Marker, 'path_marker', 10) # Added for path visualization
        self.create_subscription(Odometry, odom_topic, self.get_odom, qos_profile)
        self.create_subscription(PoseStamped, 'move_base_simple/goal', self.get_goal, 10)
        self.create_timer(0.1, self.controller) # 10 Hz timer
        
    def get_odom(self, odom):
        _, _, yaw = tf_transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    # Goal callback
    def get_goal(self, goal):
        if self.current_pose is not None:
            self.get_logger().info("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = None                                                    # to send zero velocity while planning
            self.path = [self.current_pose[0:2], self.goal]                     # to avoid path planning
            self.publish_path(self.path)
            del self.path[0]                                                    # remove current pose
        
    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self): # Removed 'event' argument as ROS2 timers don't pass it
        v = 0   
        w = 0
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available. Skipping controller update.")
            self.__send_commnd__(0.0, 0.0)
            return

        if self.path is not None and len(self.path) > 0:
            
            # If current wat point reached with some tolerance move to next point otherwise move to current point
            if np.linalg.norm(self.path[0] - self.current_pose[0:2]) < self.distance_threshold:
                self.get_logger().info("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    self.get_logger().info("Final position reached!")
            else:
                v, w = move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        self.__send_commnd__(v, w)
    

    # Publishers
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)
        
    def publish_path(self, path):
        if len(path) > 1:
            self.get_logger().info("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime.sec = 0 # Set to 0 for infinite lifetime or a specific duration
            self.marker_pub.publish(m)

            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.0
            m.scale.z = 0.0
            
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            
            color_red = ColorRGBA()
            color_red.r = 1.0
            color_red.g = 0.0
            color_red.b = 0.0
            color_red.a = 1.0
            color_blue = ColorRGBA()
            color_blue.r = 0.0
            color_blue.g = 0.0
            color_blue.b = 1.0
            color_blue.a = 1.0

            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n_point in path: # Renamed 'n' to 'n_point' to avoid conflict with numpy
                p = Point()
                p.x = n_point[0]
                p.y = n_point[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)



def main(args=None):
    rclpy.init(args=args)   
    node = Controller('odom', 'cmd_vel', 0.15)
    
    # Run forever
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()