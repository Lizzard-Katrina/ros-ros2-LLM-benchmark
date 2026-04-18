#!/usr/bin/env python
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import ExternalShutdownException
import random
import copy
import sys


class ManageObject(Node):
    def __init__(self, models_path):
        super().__init__('manage_objects')
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

        self.pub_set_model_state = self.create_publisher(Pose, 'set_model_state', 10)
        self.srv_check = self.create_service(Trigger, 'check_object', self.handle_check_object)
        self.srv_take = self.create_service(Trigger, 'get_object', self.handle_take_object)
        self.srv_let = self.create_service(Trigger, 'let_object', self.handle_let_object)

        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.iterate)

        if not self.setup_escenario():
            exit()

    def spawn_model(self, model_name, model_xml, p):
        client = self.create_client(SpawnEntity, 'spawn_entity')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn_entity service not available, waiting...')
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = model_xml
        req.initial_pose = Pose()
        req.initial_pose.position.x = p[0]
        req.initial_pose.position.y = p[1]
        req.initial_pose.position.z = 0.2
        future = client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info('Service call failed %r' % (e,))
                else:
                    return response.success
                break

    def setup_escenario(self):
        self.beer_loc = self.locations[random.randint(
            0, len(self.locations)-1)]
        if not self.spawn_model('beer', self.model_beer, self.beer_loc):
            return False
        while self.coke_loc is None or self.coke_loc == self.beer_loc:
            self.coke_loc = self.locations[random.randint(
                0, len(self.locations)-1)]
        return self.spawn_model('coke', self.model_coke, self.coke_loc)

    def handle_check_object(self, req, resp):
        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            resp.success = True
            resp.message = 'beer'
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            resp.success = True
            resp.message = 'coke'
        else:
            resp.success = False
            resp.message = ''
        return resp

    def handle_take_object(self, req, resp):
        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            self.beer_on_robot = True
            resp.success = True
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            self.coke_on_robot = True
            resp.success = True
        else:
            self.get_logger().info("Error! No objects close")
            resp.success = False
        return resp

    def handle_let_object(self, req, resp):
        if self.beer_on_robot:
            self.beer_on_robot = False
            resp.success = True
            model_state = Pose()
            model_state.position.x = self.robot_pose[0] - 0.25
            model_state.position.y = self.robot_pose[1] - 0.25
            model_state.position.z = 0.2
            self.pub_set_model_state.publish(model_state)

        elif self.coke_on_robot:
            self.coke_on_robot = False
            resp.success = True
            model_state = Pose()
            model_state.position.x = self.robot_pose[0] + 0.25
            model_state.position.y = self.robot_pose[1] + 0.25
            model_state.position.z = 0.2
            self.pub_set_model_state.publish(model_state)

        else:
            self.get_logger().info("Error! No objects grasped.")
            resp.success = False
        return resp

    def odom_callback(self, data):
        self.robot_pose = (data.pose.pose.position.x,
                           data.pose.pose.position.y)

    def distance(self, p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

    def iterate(self):
        if self.coke_on_robot:
            model_state = Pose()
            model_state.position.x = self.robot_pose[0] 
            model_state.position.y = self.robot_pose[1]
            model_state.position.z = 0.2
            self.pub_set_model_state.publish(model_state)
            self.coke_loc = copy.copy(self.robot_pose)
        elif self.beer_on_robot:
            model_state = Pose()
            model_state.position.x = self.robot_pose[0]
            model_state.position.y = self.robot_pose[1]
            model_state.position.z = 0.2
            self.pub_set_model_state.publish(model_state)
            self.beer_loc = copy.copy(self.robot_pose)


def main(args=None):
    rclpy.init(args=args)
    models_path = './'
    if len(sys.argv) >= 2:
        models_path = sys.argv[1]
    print("Path: ", models_path)
    manage_object = ManageObject(models_path)
    try:
        rclpy.spin(manage_object)
    except KeyboardInterrupt:
        manage_object.get_logger().info('Keyboard Interrupt (SIGINT)')
    except ExternalShutdownException:
        manage_object.get_logger().info('Received shutdown request')
    finally:
        manage_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
