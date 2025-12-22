#!/usr/bin/env python

import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest

# Behavior for calling `check_object` task
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    def setup(self):
        rospy.wait_for_service('/manage_objects/check_object')
        self.server = <FILL>  # LLM should fill: ServiceProxy to /manage_objects/check_object

    def initialise(self):
        <FILL>

    def update(self):
        resp = <FILL>  # Call service and set blackboard.object_name
        if resp.success:
            self.blackboard.object_name = resp.message
            return py_trees.common.Status.SUCCESS
        else:
            return <FILL>  # FAILURE

    def terminate(self, new_status):
        <FILL>

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def setup(self):
        rospy.wait_for_service('/manage_objects/get_object')
        self.server = <FILL>  # ServiceProxy

    def update(self):
        resp = <FILL>
        return <FILL>

# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def setup(self):
        rospy.wait_for_service('/manage_objects/let_object')
        self.server = <FILL>

    def update(self):
        resp = <FILL>
        return <FILL>

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    let_object = LetObject("let_object")

    # TODO: Build behavior tree <FILL>
