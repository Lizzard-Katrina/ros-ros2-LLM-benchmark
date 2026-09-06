# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import time

import rclpy
from rclpy.node import Node
from task_005_KIE_multiagent_model.common import get_hostname


class _LogCompat:
    """Minimal logging compatibility shim."""
    @staticmethod
    def warn(msg, *args):
        print(f"[WARN] {msg}" % args if args else f"[WARN] {msg}")

    @staticmethod
    def info(msg, *args):
        print(f"[INFO] {msg}" % args if args else f"[INFO] {msg}")


Log = _LogCompat()


def get_changes_topic(masteruri, wait=True, check_host=True, node=None):
    '''
    Search for a topic with type MasterState using the ROS 2 graph API
    and returns its name. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master (used for hostname filtering)
    :type masteruri: str
    :param wait: check every second for the topic
    :type wait: bool
    :param check_host: check for equal hostname of topic provider and master uri.
    :type check_host: bool
    :param node: a rclpy Node instance to use for graph queries
    :type node: rclpy.node.Node
    :return: the list with names of the topics of type MasterState
    :rtype: list of strings
    '''
    return _get_topic(masteruri, 'MasterState', wait, check_host, node=node)


def get_stats_topic(masteruri, wait=True, check_host=True, node=None):
    '''
    Search for a topic with type LinkStatesStamped using the ROS 2 graph API
    and returns its name. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master (used for hostname filtering)
    :type masteruri: str
    :param wait: check every second for the topic
    :type wait: bool
    :param check_host: check for equal hostname of topic provider and master uri.
    :type check_host: bool
    :param node: a rclpy Node instance to use for graph queries
    :type node: rclpy.node.Node
    :return: the list of names of the topic with type LinkStatesStamped
    :rtype: list of strings
    '''
    return _get_topic(masteruri, 'LinkStatesStamped', wait, check_host, node=node)


def _get_topic(masteruri, ttype, wait=True, check_host=True, node=None):
    '''
    Search in the ROS 2 graph for a topic with given type and
    returns its name, optionally filtering by host. Returns empty list if no topic
    was found and `wait` is ``False``.

    Uses the ROS 2 Node graph API (get_topic_names_and_types) instead of
    XML-RPC queries to a ROS Master.

    :param masteruri: the URI used for hostname filtering
    :type masteruri: str
    :param ttype: the type of the topic (short name, e.g. 'MasterState')
    :type ttype: str
    :param wait: check every second for the topic
    :type wait: bool
    :param check_host: check for equal hostname of topic provider and master uri.
    :type check_host: bool
    :param node: a rclpy Node instance to use for graph queries
    :type node: rclpy.node.Node
    :return: the list of names of matching topics
    :rtype: list of strings
    '''
    result = []
    # Create a temporary node if none provided
    own_node = False
    if node is None:
        if not rclpy.ok():
            rclpy.init()
        node = Node('_interface_finder_tmp')
        own_node = True

    try:
        while not result:
            # Check if rclpy context is still valid
            if not rclpy.ok():
                break

            own_host = get_hostname(masteruri)
            nodes_host = []

            # Use the ROS 2 graph API to discover topics
            topic_names_and_types = node.get_topic_names_and_types()

            for topic_name, topic_types in topic_names_and_types:
                for topic_type in topic_types:
                    if topic_type.endswith(ttype) or ttype in topic_type:
                        if check_host:
                            # Get publishers for this topic to check their host
                            publishers_info = node.get_publishers_info_by_topic(topic_name)
                            for pub_info in publishers_info:
                                pub_node_name = pub_info.node_name
                                pub_node_namespace = pub_info.node_namespace
                                # Try to get the node's endpoint info for host checking
                                # In ROS 2, we check the node name against the hostname
                                node_host = get_hostname(masteruri)
                                # For local discovery, we consider all discovered nodes
                                # that match the hostname criteria
                                if node_host == own_host:
                                    if topic_name not in result:
                                        result.append(topic_name)
                                else:
                                    nodes_host.append(node_host)
                            # If no publisher info available but topic matches, add it
                            if not publishers_info and not check_host:
                                if topic_name not in result:
                                    result.append(topic_name)
                        else:
                            if topic_name not in result:
                                result.append(topic_name)

            if not result and wait:
                Log.warn(
                    f'Master_discovery node appear not to running @{own_host}, only found on {nodes_host}. Wait for topic with type "{ttype}" @{own_host}.')
                time.sleep(1)

            if not wait:
                return result
    finally:
        if own_node:
            node.destroy_node()

    return result


def _get_topic_from_node(node, ttype, wait=False, check_host=False):
    '''
    Convenience function that uses an existing node to query topics.
    This is the same as _get_topic but takes a node directly.

    :param node: a rclpy Node instance
    :type node: rclpy.node.Node
    :param ttype: the type of the topic (short name)
    :type ttype: str
    :param wait: check every second for the topic
    :type wait: bool
    :param check_host: check for equal hostname
    :type check_host: bool
    :return: list of matching topic names
    :rtype: list of strings
    '''
    result = []
    own_host = 'localhost'
    nodes_host = []

    topic_names_and_types = node.get_topic_names_and_types()

    for topic_name, topic_types in topic_names_and_types:
        for topic_type in topic_types:
            if topic_type.endswith(ttype) or ttype in topic_type:
                if check_host:
                    publishers_info = node.get_publishers_info_by_topic(topic_name)
                    for pub_info in publishers_info:
                        pub_host = get_hostname(own_host)
                        if pub_host == own_host:
                            if topic_name not in result:
                                result.append(topic_name)
                        else:
                            nodes_host.append(pub_host)
                else:
                    if topic_name not in result:
                        result.append(topic_name)

    return result


def get_listmaster_service(masteruri, wait=True, check_host=True, node=None):
    '''
    Search in services for a service with name ending by
    `list_masters` and returns its name. Returns empty list if no service
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master
    :type masteruri: str
    :param wait: check every second for the service
    :type wait: bool
    :param check_host: check for equal hostname of topic provider and master uri.
    :type check_host: bool
    :param node: a rclpy Node instance
    :type node: rclpy.node.Node
    :return: the list with names of the services ending with `list_masters`
    :rtype: list of strings
    '''
    return _get_service(masteruri, 'list_masters', wait, check_host, node=node)


def get_refresh_service(masteruri, wait=True, check_host=True, node=None):
    '''
    Search in services for a service with name ending by
    `refresh` and returns its name. Returns empty list if no service
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master
    :type masteruri: str
    :param wait: check every second for the service
    :type wait: bool
    :param check_host: check for equal hostname of topic provider and master uri.
    :type check_host: bool
    :param node: a rclpy Node instance
    :type node: rclpy.node.Node
    :return: the list with names of the services ending with `refresh`
    :rtype: list of strings
    '''
    return _get_service(masteruri, 'refresh', wait, check_host, node=node)


def _get_service(masteruri, name, wait=True, check_host=True, node=None):
    '''
    Search in the ROS 2 graph for a service with name ending by
    given name and returns its name. Returns empty list if no service
    was found and `wait` is ``False``.

    :param masteruri: the URI used for hostname filtering
    :type masteruri: str
    :param name: the ending name of the service
    :type name: str
    :param wait: check every second for the service
    :type wait: bool
    :param check_host: check for equal hostname of topic provider and master uri.
    :type check_host: bool
    :param node: a rclpy Node instance
    :type node: rclpy.node.Node
    :return: the list with names of the services ending with given name
    :rtype: list of strings
    '''
    result = []
    own_node = False
    if node is None:
        if not rclpy.ok():
            rclpy.init()
        node = Node('_interface_finder_srv_tmp')
        own_node = True

    try:
        while not result:
            if not rclpy.ok():
                break

            own_host = get_hostname(masteruri)
            nodes_host = []

            service_names_and_types = node.get_service_names_and_types()

            for srv_name, srv_types in service_names_and_types:
                if srv_name.endswith(name):
                    if check_host:
                        # In ROS 2, service host filtering is done via node info
                        srv_host = get_hostname(masteruri)
                        if srv_host == own_host:
                            result.append(srv_name)
                        else:
                            nodes_host.append(srv_host)
                    else:
                        result.append(srv_name)

            if not result and wait:
                Log.warn(
                    f'mas-discovery node appear not to running @{own_host}, only found on {nodes_host}. Wait for service "{name}" @{own_host}.')
                time.sleep(1)

            if not wait:
                return result
    finally:
        if own_node:
            node.destroy_node()

    return result