#!/usr/bin/env python3

import py_trees
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(CheckObject, self).__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, 'check_object')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.logger.debug(
                "  %s [CheckObject::setup() Service not available!]" % self.name)
        else:
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        # 1. Log the service call to 'check_object'.
        self.logger.debug(
            "  {}: call service check_object".format(self.name))
        # 2. Call the service using the client created in setup().
        request = Trigger.Request()
        future = self.client.call_async(request)
        # 3. Since PyTrees ticks are synchronous, handle the rclpy future to get the result.
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        # 4. If success: write 'resp.message' to self.blackboard.object_name and return SUCCESS.
        if future.done() and future.result() is not None:
            resp = future.result()
            if resp.success:
                self.blackboard.object_name = resp.message
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        # 5. Otherwise: return FAILURE.
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(GetObject, self).__init__(name)
        self.node = node

    def setup(self, **kwargs):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, 'get_object')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.logger.debug(
                "  %s [GetObject::setup() Service not available!]" % self.name)
        else:
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service get_object".format(self.name))
            request = Trigger.Request()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                resp = future.result()
                if resp.success:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.FAILURE
        except Exception:
            self.logger.debug(
                "  {}: Error calling service get_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(LetObject, self).__init__(name)
        self.node = node

    def setup(self, **kwargs):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, 'let_object')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.logger.debug(
                "  %s [LetObject::setup() Service not available!]" % self.name)
        else:
            self.logger.debug(
                "  %s [LetObject::setup() Server connected!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service let_object".format(self.name))
            request = Trigger.Request()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                resp = future.result()
                if resp.success:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.FAILURE
        except Exception:
            self.logger.debug(
                "  {}: Error calling service let_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


def main(args=None):
    rclpy.init(args=args)
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    node = Node("behavior_trees")

    # Create Behaviors
    check_object = CheckObject("check_object", node)
    get_object = GetObject("get_object", node)
    let_object = LetObject("let_object", node)

    # Setup behaviors
    check_object.setup()
    get_object.setup()
    let_object.setup()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()