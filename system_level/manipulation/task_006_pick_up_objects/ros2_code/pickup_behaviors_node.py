#!/usr/bin/env python3

import py_trees
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_trees_node')

# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(CheckObject, self).__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, '/manage_objects/check_object')
        if not self.client.wait_for_service(timeout_sec=3.0):
            self.logger.debug("  %s [CheckObject::setup() ERROR!]" % self.name)
        else:
            self.logger.debug("  %s [CheckObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  {}: call service /manage_objects/check_object".format(self.name))
        req = Trigger.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            resp = future.result()
            if resp.success:
                self.blackboard.object_name = resp.message
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(GetObject, self).__init__(name)
        self.node = node

    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, '/manage_objects/get_object')
        if not self.client.wait_for_service(timeout_sec=3.0):
            self.logger.debug("  %s [GetObject::setup() ERROR!]" % self.name)
        else:
            self.logger.debug("  %s [GetObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/get_object".format(self.name))
            req = Trigger.Request()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None and future.result().success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.debug(
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

    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, '/manage_objects/let_object')
        if not self.client.wait_for_service(timeout_sec=3.0):
            self.logger.debug("  %s [LetObject::setup() ERROR!]" % self.name)
        else:
            self.logger.debug("  %s [LetObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/let_object".format(self.name))
            req = Trigger.Request()
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None and future.result().success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/let_object: {}".format(self.name, e))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


def main(args=None):
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rclpy.init(args=args)
    node = BehaviorNode()

    # Create Behaviors
    check_object = CheckObject("check_object", node)
    get_object = GetObject("get_object", node)
    let_object = LetObject("let_object", node)
    
    check_object.setup()
    get_object.setup()
    let_object.setup()
    
    # fill the rest of the code here ...
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()