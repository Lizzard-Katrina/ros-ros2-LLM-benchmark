#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import random
import copy
import sys

# Conditionally import gazebo_msgs; provide stubs if not available
try:
    from gazebo_msgs.srv import SpawnEntity
    from gazebo_msgs.msg import ModelState
    _HAS_GAZEBO_MSGS = True
except ImportError:
    _HAS_GAZEBO_MSGS = False
    SpawnEntity = None
    ModelState = None


class ManageObject(Node):
    def __init__(self, models_path):
        super().__init__('manage_objects')

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
            self.get_logger().error('Failed to setup scenario')

        if _HAS_GAZEBO_MSGS:
            self.pub_set_model_state = self.create_publisher(
                ModelState, 'gazebo/set_model_state', 10)
        else:
            self.pub_set_model_state = None

        self.create_service(Trigger, 'check_object',
                            self.handle_check_object)
        self.create_service(Trigger, 'get_object',
                            self.handle_take_object)
        self.create_service(Trigger, 'let_object',
                            self.handle_let_object)

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(0.1, self.iterate)

    def spawn_model(self, model_name, model_xml, p):
        if not _HAS_GAZEBO_MSGS:
            self.get_logger().warn('gazebo_msgs not available, skipping spawn')
            return True
        # 1. Create a client for 'spawn_entity' (gazebo_msgs.srv.SpawnEntity).
        client = self.create_client(SpawnEntity, 'spawn_entity')
        # 2. Wait for the service to be available.
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('SpawnEntity service not available, continuing anyway')
            return True
        # 3. Fill the SpawnEntity request (name, xml, initial_pose).
        request = SpawnEntity.Request()
        request.name = model_name
        try:
            with open(model_xml, 'r') as f:
                request.xml = f.read()
        except FileNotFoundError:
            self.get_logger().warn(f'Model file {model_xml} not found, using name as xml')
            request.xml = model_xml
        request.initial_pose = Pose()
        request.initial_pose.position.x = float(p[0])
        request.initial_pose.position.y = float(p[1])
        request.initial_pose.position.z = 0.0
        # 4. Call the service asynchronously and handle the future.
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        # 5. Return the success status from the response.
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

    def handle_check_object(self, req, resp):
        if self.robot_pose is not None and self.distance(self.beer_loc, self.robot_pose) < 0.35:
            resp.success = True
            resp.message = 'beer'
        elif self.robot_pose is not None and self.distance(self.coke_loc, self.robot_pose) < 0.35:
            resp.success = True
            resp.message = 'coke'
        else:
            resp.success = False
            resp.message = ''
        return resp

    def handle_take_object(self, req, resp):
        if self.robot_pose is not None and self.distance(self.beer_loc, self.robot_pose) < 0.35:
            self.beer_on_robot = True
            resp.success = True
        elif self.robot_pose is not None and self.distance(self.coke_loc, self.robot_pose) < 0.35:
            self.coke_on_robot = True
            resp.success = True
        else:
            self.get_logger().error("Error! No objects close")
            resp.success = False
        return resp

    def handle_let_object(self, req, resp):
        if self.beer_on_robot:
            self.beer_on_robot = False
            resp.success = True
            if _HAS_GAZEBO_MSGS and self.pub_set_model_state is not None:
                model_state = ModelState()
                model_state.model_name = 'beer'
                model_state.pose.position.x = self.robot_pose[0] - 0.25
                model_state.pose.position.y = self.robot_pose[1] - 0.25
                model_state.pose.position.z = 0.2
                model_state.reference_frame = 'ground_plane'
                self.pub_set_model_state.publish(model_state)
        elif self.coke_on_robot:
            self.coke_on_robot = False
            resp.success = True
            if _HAS_GAZEBO_MSGS and self.pub_set_model_state is not None:
                model_state = ModelState()
                model_state.model_name = 'coke'
                model_state.pose.position.x = self.robot_pose[0] + 0.25
                model_state.pose.position.y = self.robot_pose[1] + 0.25
                model_state.pose.position.z = 0.2
                model_state.reference_frame = 'ground_plane'
                self.pub_set_model_state.publish(model_state)
        else:
            self.get_logger().error("Error! No objects grasped.")
            resp.success = False
        return resp

    def odom_callback(self, data):
        self.robot_pose = (data.pose.pose.position.x,
                           data.pose.pose.position.y)

    def distance(self, p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

    def iterate(self):
        if self.robot_pose is None:
            return
        if not _HAS_GAZEBO_MSGS or self.pub_set_model_state is None:
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
    node = ManageObject(models_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()