#!/usr/bin/env python
import rospy
import yaml
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

rospy.init_node('motion_planning_param_server_v03')

robot = RobotCommander()
scene = PlanningSceneInterface()
group = MoveGroupCommander("arm")

# TODO 1: load planner configuration from YAML file
# end: 

group.set_num_planning_attempts(10)

# execute a sample motion
group.set_joint_value_target([0, -1.0, 1.2, 0.5, 0, 1.0])
plan = group.plan()
group.execute(plan, wait=True)
