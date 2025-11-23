#!/usr/bin/env python3
import rospy
# TODO: import correct service type
from gazebo_msgs.srv import ________   # SpawnModel
from geometry_msgs.msg import Pose

def spawn_model():
    # TODO: fill in the spawn service name
    rospy.wait_for_service("__________")
    
    # TODO: complete the entire service call block
    try:
        # TODO: create ServiceProxy with correct service name & type
        spawn_srv = ________
        
        # TODO: define model_name, SDF/XML content, pose, reference frame
        model_name = "__________"
        model_xml = "__________"
        pose = ________  
        reference_frame = "__________"
        
        # TODO: call the service and handle response correctly
        resp = ________
        rospy.loginfo("Model spawned successfully: ________")  # maybe resp status
        
    except Exception as e:
        # TODO: handle exceptions properly
        rospy.logerr("Spawn failed: ________")

if __name__ == "__main__":
    rospy.init_node("spawn_model_node")
    spawn_model()
