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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import random
import copy
import sys
import os

class ManageObject(Node):
    def __init__(self, models_path):
        super().__init__('spawn_model')
        
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
            self.get_logger().error("Failed to setup scenario")
            return
            
        self.pub_set_model_state = self.create_publisher(
            ModelState, '/gazebo/set_model_state', 1)
        
        self.server_check = self.create_service(
            Trigger, '~/check_object', self.handle_check_object)
        self.server_take = self.create_service(
            Trigger, '~/get_object', self.handle_take_object)
        self.server_let = self.create_service(
            Trigger, '~/let_object', self.handle_let_object)

        self.subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.1, self.iterate)

    def spawn_model(self, model_name, model_xml, p):
        # TODO
        ## 1. Create a client for '/spawn_entity' (gazebo_msgs.srv.SpawnEntity).
        # 2. Wait for the service to be available.
        # 3. Fill the SpawnEntity request (name, xml, initial_pose).
        # 4. Call the service asynchronously and handle the future.
        # 5. Return the success status from the response.
        # END OF TODO
        client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = open(model_xml, 'r').read()
        
        initial_pose = Pose()
        initial_pose.position.x = p[0]
        initial_pose.position.y = p[1]
        initial_pose.position.z = 0.2
        initial_pose.orientation.w = 1.0
        req.initial_pose = initial_pose
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully spawned {model_name}")
                return True
            else:
                self.get_logger().error(f"Failed to spawn {model_name}: {response.status_message}")
                return False
        else:
            self.get_logger().error("Service call failed")
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

    def handle_check_object(self, request, response):
        if self.robot_pose is None:
            response.success = False
            response.message = ''
            return response
            
        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            response.success = True
            response.message = 'beer'
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            response.success = True
            response.message = 'coke'
        else:
            response.success = False
            response.message = ''
        return response

    def handle_take_object(self, request, response):
        if self.robot_pose is None:
            response.success = False
            return response
            
        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            self.beer_on_robot = True
            response.success = True
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            self.coke_on_robot = True
            response.success = True
        else:
            self.get_logger().info("Error! No objects close")
            response.success = False
        return response

    def handle_let_object(self, request, response):
        if self.beer_on_robot:
            self.beer_on_robot = False
            response.success = True
            model_state = ModelState()
            model_state.model_name = 'beer'
            model_state.pose.position.x = self.robot_pose[0] - 0.25
            model_state.pose.position.y = self.robot_pose[1] - 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)

        elif self.coke_on_robot:
            self.coke_on_robot = False
            response.success = True
            model_state = ModelState()
            model_state.model_name = 'coke'
            model_state.pose.position.x = self.robot_pose[0] + 0.25
            model_state.pose.position.y = self.robot_pose[1] + 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)

        else:
            self.get_logger().info("Error! No objects grasped.")
            response.success = False
        return response

    def odom_callback(self, data):
        self.robot_pose = (data.pose.pose.position.x,
                           data.pose.pose.position.y)

    def distance(self, p1, p2):
        if p2 is None:
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


def main(args=None):
    models_path = './'
    print(sys.argv)
    if len(sys.argv) >= 2:
        models_path = sys.argv[1]
     
    print("Path: ", models_path)
    rclpy.init(args=args)
    check_object = ManageObject(models_path)
    executor = MultiThreadedExecutor()
    executor.add_node(check_object)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        check_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

[FILENAME: pickup_behaviors_node.py]
#!/usr/bin/env python3

import py_trees
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.executors import SingleThreadedExecutor

# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)
        self.node = None
        self.client = None

    def setup(self, **kwargs):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        self.node = kwargs.get('node', None)
        if self.node is None:
            self.node = rclpy.create_node('check_object_client')
        
        self.client = self.node.create_client(Trigger, '/manage_objects/check_object')
        
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug("  %s [CheckObject::setup() Service not available]" % self.name)
        else:
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name)

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
        self.logger.debug("  {}: call service /manage_objects/check_object".format(self.name))
        
        req = Trigger.Request()
        future = self.client.call_async(req)
        
        # Spin the node until the future is done
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        if future.result() is not None:
            resp = future.result()
            if resp.success:
                self.blackboard.object_name = resp.message
                self.logger.debug("  {}: Object found: {}".format(self.name, resp.message))
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.debug("  {}: No object found".format(self.name))
                return py_trees.common.Status.FAILURE
        else:
            self.logger.debug("  {}: Service call failed".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GetObject, self).__init__(name)
        self.node = None
        self.client = None

    def setup(self, **kwargs):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        self.node = kwargs.get('node', None)
        if self.node is None:
            self.node = rclpy.create_node('get_object_client')
            
        self.client = self.node.create_client(Trigger, '/manage_objects/get_object')
        
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug("  %s [GetObject::setup() Service not available]" % self.name)
        else:
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/get_object".format(self.name))
            req = Trigger.Request()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.result() is not None:
                resp = future.result()
                if resp.success:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/get_object: {}".format(self.name, str(e)))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)
        self.node = None
        self.client = None

    def setup(self, **kwargs):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        self.node = kwargs.get('node', None)
        if self.node is None:
            self.node = rclpy.create_node('let_object_client')
            
        self.client = self.node.create_client(Trigger, '/manage_objects/let_object')
        
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug("  %s [LetObject::setup() Service not available]" % self.name)
        else:
            self.logger.debug(
                "  %s [LetObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/let_object".format(self.name))
            req = Trigger.Request()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.result() is not None:
                resp = future.result()
                if resp.success:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/let_object: {}".format(self.name, str(e)))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


def create_tree():
    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    let_object = LetObject("let_object")
    
    # Simple sequence: Check -> Get -> Let
    root = py_trees.composites.Sequence("PickAndPlace", memory=True)
    root.add_child(check_object)
    root.add_child(get_object)
    root.add_child(let_object)
    
    return root

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rclpy.init()
    
    node = rclpy.create_node("behavior_trees")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # Create Behaviors
    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    let_object = LetObject("let_object")
    
    # fill the rest of the code here ...
    root = py_trees.composites.Sequence("MainSequence", memory=True)
    root.add_child(check_object)
    root.add_child(get_object)
    root.add_child(let_object)
    
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15, node=node)
    
    try:
        while rclpy.ok():
            tree.tick()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()

[FILENAME: turtlebot_controller_node.py]
#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from geometry_msgs.msg import PoseStamped
import math

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
        # TODO
        # 1. Store the node and distance_threshold.
        # 2. Create a Publisher for 'cmd_vel_topic' (Twist, qos=10).
        # 3. Create a Subscriber for 'odom_topic' (Odometry) and '/move_base_simple/goal' (PoseStamped).
        # 4. IMPORTANT: Ensure topic/frame names follow ROS 2 convention (no leading slashes).
        # 5. Create a Timer (10Hz) to call 'self.controller'.    
        # END OF TODO
        self.distance_threshold = distance_threshold
        self.current_pose = None
        self.goal = None
        self.path = None
        self.Kv = 0.5
        self.Kw = 0.5
        self.v_max = 0.5
        self.w_max = 1.0
        
        # Remove leading slashes for ROS2 topic names
        cmd_vel_topic_clean = cmd_vel_topic.lstrip('/')
        odom_topic_clean = odom_topic.lstrip('/')
        goal_topic_clean = 'move_base_simple/goal'
        
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic_clean, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic_clean, self.get_odom, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, goal_topic_clean, self.get_goal, 10)
        self.marker_pub = self.create_publisher(Marker, 'path_marker', 10)
        
        self.timer = self.create_timer(0.1, self.controller)

    def get_odom(self, odom):
        # Convert quaternion to euler
        orientation = odom.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])
    
    # Goal callback
    def get_goal(self, goal):
        if self.current_pose is not None:
            self.get_logger().info("New goal received: ({}, {})".format(goal.pose.position.x, goal.pose.position.y))
            self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
            self.path = None                                                    # to send zero velocity while planning
            self.path = [self.current_pose[0:2].tolist(), self.goal.tolist()]                     # to avoid path planning
            self.publish_path(self.path)
            del self.path[0]                                                    # remove current pose
        
    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self):
        v = 0   
        w = 0
        if self.path is not None and len(self.path) > 0:
            
            # If current wat point reached with some tolerance move to next point otherwise move to current point
            if np.linalg.norm(np.array(self.path[0]) - self.current_pose[0:2]) < self.distance_threshold:
                self.get_logger().info("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    self.get_logger().info("Final position reached!")
            else:
                v, w = move_to_point(self.current_pose, np.array(self.path[0]), self.Kv, self.Kw)
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
            self.get_logger().info("Publish path!")
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.ns = 'path'
            m.action = Marker.DELETE
            m.lifetime = Duration(seconds=0).to_msg()
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
            p.x = float(self.current_pose[0])
            p.y = float(self.current_pose[1])
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in path:
                p = Point()
                p.x = float(n[0])
                p.y = float(n[1])
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
            
            self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)   
    node = Controller('/odom', '/cmd_vel', 0.15)
    
    # Run forever
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()