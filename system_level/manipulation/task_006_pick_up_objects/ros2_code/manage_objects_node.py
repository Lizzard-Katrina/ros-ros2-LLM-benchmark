#!/usr/bin/env python3
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node
import random
import copy
import sys
import os


class ManageObject(Node):
    def __init__(self, models_path):
        super().__init__('manage_objects_node')
        
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
            self.get_logger().error("Failed to setup scenario")
            sys.exit(1)
            
        self.pub_set_model_state = self.create_publisher(
            ModelState, '/gazebo/set_model_state', 1)
            
        self.server_check = self.create_service(Trigger, '~/check_object',
                                     self.handle_check_object)
        self.server_take = self.create_service(Trigger, '~/get_object',
                                    self.handle_take_object)
        self.server_let = self.create_service(Trigger, '~/let_object',
                                   self.handle_let_object)

        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.iterate)

    def spawn_model(self, model_name, model_xml, p):
        client = self.create_client(SpawnEntity, '/spawn_entity')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = SpawnEntity.Request()
        req.name = model_name
        try:
            with open(model_xml, 'r') as f:
                req.xml = f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read model xml: {e}")
            return False
            
        req.initial_pose.position.x = float(p[0])
        req.initial_pose.position.y = float(p[1])
        req.initial_pose.position.z = 0.0
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        return False

    def setup_escenario(self):
        self.beer_loc = self.locations[random.randint(
            0, len(self.locations)-1)]
        self.spawn_model('beer', self.model_beer, self.beer_loc)
        while self.coke_loc is None or self.coke_loc == self.beer_loc:
            self.coke_loc = self.locations[random.randint(
                0, len(self.locations)-1)]
        return self.spawn_model('coke', self.model_coke, self.coke_loc)

    def handle_check_object(self, req, res):
        if self.robot_pose is None:
            res.success = False
            res.message = ''
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
            return res
            
        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            self.beer_on_robot = True
            res.success = True
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            self.coke_on_robot = True
            res.success = True
        else:
            self.get_logger().error("Error! No objects close")
            res.success = False
        return res

    def handle_let_object(self, req, res):
        if self.robot_pose is None:
            res.success = False
            return res
            
        if self.beer_on_robot:
            self.beer_on_robot = False
            res.success = True
            model_state = ModelState()
            model_state.model_name = 'beer'
            model_state.pose.position.x = self.robot_pose[0] - 0.25
            model_state.pose.position.y = self.robot_pose[1] - 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)

        elif self.coke_on_robot:
            self.coke_on_robot = False
            res.success = True
            model_state = ModelState()
            model_state.model_name = 'coke'
            model_state.pose.position.x = self.robot_pose[0] + 0.25
            model_state.pose.position.y = self.robot_pose[1] + 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)

        else:
            self.get_logger().error("Error! No objects grasped.")
            res.success = False
        return res

    def odom_callback(self, data):
        self.robot_pose = (data.pose.pose.position.x,
                           data.pose.pose.position.y)

    def distance(self, p1, p2):
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
    rclpy.init(args=args)
    models_path = './'
    print(sys.argv)
    if len(sys.argv) >= 2:
        models_path = sys.argv[1]
     
    print("Path: ", models_path)
    check_object = ManageObject(models_path)
    rclpy.spin(check_object)
    check_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()