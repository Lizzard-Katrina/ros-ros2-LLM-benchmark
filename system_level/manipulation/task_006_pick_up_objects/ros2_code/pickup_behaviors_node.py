#!/usr/bin/env python

import py_trees
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile


# Behavior for calling `check_object` task and if True, store object name to Blackboard
class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "object_name", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, 'manage_objects/check_object')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug("  %s [CheckObject::setup() Service not available!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        req = Trigger.Request()
        future = self.client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.logger.debug("  %s [CheckObject::update() ERROR!]" % self.name)
                else:
                    if response.success:
                        self.blackboard.object_name = response.message
                        return py_trees.common.Status.SUCCESS
                    else:
                        return py_trees.common.Status.FAILURE
                break
    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, 'manage_objects/get_object')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug("  %s [GetObject::setup() Service not available!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)

    def update(self):
        try:
            req = Trigger.Request()
            future = self.client.call_async(req)
            while rclpy.ok():
                rclpy.spin_once(self.node)
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        self.logger.debug("  %s [GetObject::update() ERROR!]" % self.name)
                    else:
                        if response.success:
                            return py_trees.common.Status.SUCCESS
                        else:
                            return py_trees.common.Status.FAILURE
                    break
        except:
            self.logger.debug("  %s [GetObject::update() ERROR!]" % self.name)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        self.client = self.node.create_client(Trigger, 'manage_objects/let_object')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.debug("  %s [LetObject::setup() Service not available!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        try:
            req = Trigger.Request()
            future = self.client.call_async(req)
            while rclpy.ok():
                rclpy.spin_once(self.node)
                if future.done():
                    try:
                        response = future.result()
                    except Exception as e:
                        self.logger.debug("  %s [LetObject::update() ERROR!]" % self.name)
                    else:
                        if response.success:
                            return py_trees.common.Status.SUCCESS
                        else:
                            return py_trees.common.Status.FAILURE
                    break
        except:
            self.logger.debug("  %s [LetObject::update() ERROR!]" % self.name)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))


class BehaviorTree(Node):
    def __init__(self):
        super().__init__('behavior_tree')
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        self.check_object = CheckObject("check_object")
        self.get_object = GetObject("get_object")
        self.let_object = LetObject("let_object")
        self.root = py_trees.composites.Selector("Selector")
        self.root.add_child(self.check_object)
        self.root.add_child(self.get_object)
        self.root.add_child(self.let_object)
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self.tree.setup(timeout=15)

    def spin(self):
        while rclpy.ok():
            self.tree.tick()
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    behavior_tree = BehaviorTree()
    try:
        behavior_tree.spin()
    except KeyboardInterrupt:
        behavior_tree.get_logger().info('Keyboard Interrupt (SIGINT)')
    except ExternalShutdownException:
        behavior_tree.get_logger().info('Received shutdown request')
    finally:
        behavior_tree.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
