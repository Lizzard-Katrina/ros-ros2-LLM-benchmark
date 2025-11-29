#!/usr/bin/env python
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

rospy.init_node('motion_planning_param_server_v02')

robot = RobotCommander()
scene = PlanningSceneInterface()
group = MoveGroupCommander("arm")

# TODO : load velocity and acceleration scaling from ROS param server
# end: 

group.set_planner_id("RRTConnectkConfigDefault")
group.set_num_planning_attempts(10)

# execute a sample motion
group.set_joint_value_target([0, -1.0, 1.2, 0.5, 0, 1.0])
plan = group.plan()
group.execute(plan, wait=True)
