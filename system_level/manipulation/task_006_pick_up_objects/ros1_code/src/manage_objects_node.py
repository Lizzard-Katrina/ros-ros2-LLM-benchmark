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
        # TODO: Fill code to spawn a model in Gazebo using SpawnModel service
        # END
     
    def setup_escenario(self):
        # # TODO: Randomly place beer and coke in locations and spawn models
        # END
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
    
