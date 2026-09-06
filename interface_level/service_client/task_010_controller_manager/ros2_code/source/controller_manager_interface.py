#! /usr/bin/env python3
from __future__ import print_function
import rclpy
from rclpy.node import Node
from task_010_controller_manager.srv import ListControllerTypes, ListControllers, \
    LoadController, UnloadController, SwitchController, ReloadControllerLibraries


_node = None


def _get_node():
    global _node
    if _node is None:
        if not rclpy.ok():
            rclpy.init()
        _node = rclpy.create_node('controller_manager_interface')
    return _node


def _call_service(srv_type, srv_name, request):
    node = _get_node()
    client = node.create_client(srv_type, srv_name)
    client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    return future.result()


def list_controller_types():
    req = ListControllerTypes.Request()
    resp = _call_service(ListControllerTypes, 'controller_manager/list_controller_types', req)
    for t in resp.types:
        print(t)


def reload_libraries(force_kill, restore=False):
    node = _get_node()

    reload_client = node.create_client(ReloadControllerLibraries, 'controller_manager/reload_controller_libraries')
    reload_client.wait_for_service(timeout_sec=5.0)

    list_client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    load_client = node.create_client(LoadController, 'controller_manager/load_controller')
    switch_client = node.create_client(SwitchController, 'controller_manager/switch_controller')

    print("Restore: " + str(restore))
    if restore:
        list_client.wait_for_service(timeout_sec=5.0)
        list_req = ListControllers.Request()
        future = list_client.call_async(list_req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        originally = future.result()

    reload_req = ReloadControllerLibraries.Request()
    reload_req.force_kill = force_kill
    future = reload_client.call_async(reload_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    resp = future.result()
    if resp.ok:
        print("Successfully reloaded libraries")
        result = True
    else:
        print("Failed to reload libraries. Do you still have controllers loaded?")
        result = False

    if restore:
        load_client.wait_for_service(timeout_sec=5.0)
        switch_client.wait_for_service(timeout_sec=5.0)
        for c in originally.controller:
            load_req = LoadController.Request()
            load_req.name = c.name
            future = load_client.call_async(load_req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        to_start = [c.name for c in originally.controller if c.state == 'running']
        switch_req = SwitchController.Request()
        switch_req.start_controllers = to_start
        switch_req.stop_controllers = []
        switch_req.strictness = SwitchController.Request.BEST_EFFORT
        future = switch_client.call_async(switch_req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        print("Controllers restored to original state")
    return result


def list_controllers():
    node = _get_node()
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')
    client.wait_for_service(timeout_sec=5.0)
    req = ListControllers.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    resp = future.result()
    if len(resp.controller) == 0:
        print("No controllers are loaded in mechanism control")
    else:
        for c in resp.controller:
            hwi = c.claimed_interfaces
            print("'%s' - '%s' ( %s )" % (c.name, "+".join(hwi), c.state))


def load_controller(name):
    req = LoadController.Request()
    req.name = name
    resp = _call_service(LoadController, 'controller_manager/load_controller', req)
    if resp.ok:
        print("Loaded \'" + name + "\'")
        return True
    else:
        print("Error when loading \'" + name + "\'")
        return False


def unload_controller(name):
    req = UnloadController.Request()
    req.name = name
    resp = _call_service(UnloadController, 'controller_manager/unload_controller', req)
    if resp.ok == 1:
        print("Unloaded \'" + name + "\' successfully")
        return True
    else:
        print("Error when unloading \'" + name + "\'")
        return False


def start_controller(name):
    return start_stop_controllers(start_controllers=[name])


def start_controllers(names):
    return start_stop_controllers(start_controllers=names)


def stop_controller(name):
    return start_stop_controllers(stop_controllers=[name])


def stop_controllers(names):
    return start_stop_controllers(stop_controllers=names)


def start_stop_controllers(start_controllers=[], stop_controllers=[]):
    node = _get_node()
    client = node.create_client(SwitchController, 'controller_manager/switch_controller')
    client.wait_for_service(timeout_sec=5.0)
    req = SwitchController.Request()
    req.start_controllers = start_controllers
    req.stop_controllers = stop_controllers
    req.strictness = SwitchController.Request.STRICT
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    resp = future.result()
    if resp.ok == 1:
        if start_controllers:
            print("Started {} successfully".format(start_controllers))
        if stop_controllers:
            print("Stopped {} successfully".format(stop_controllers))
        return True
    else:
        print("Error when starting {} and stopping {}".format(start_controllers, stop_controllers))
        return False


def main():
    rclpy.init()
    list_controllers()
    rclpy.shutdown()


if __name__ == '__main__':
    main()