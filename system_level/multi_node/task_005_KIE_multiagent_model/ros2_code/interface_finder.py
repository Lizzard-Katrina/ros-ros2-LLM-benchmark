# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import time
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

import rclpy
from rclpy.node import Node
from .common import get_hostname
from fkie_mas_pylib.logging.logging import Log


def get_changes_topic(masteruri, wait=True, check_host=True):
    '''
    Search in publishers of ROS master for a topic with type `fkie_mas_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/MasterState.html>`_ and
    returns his name, if it runs on the local host. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the topics of type `fkie_mas_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/MasterState.html>`_

    :rtype: list of strings
    '''
    return _get_topic(masteruri, 'MasterState', wait, check_host)


def get_stats_topic(masteruri, wait=True, check_host=True):
    '''
    Search in publishers of ROS master for a topic with type LinkStatesStamped and
    returns his name, if it runs on the local host. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list of names of the topic with type `fkie_mas_discovery.msg.LinkStatesStamped <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/LinkStatesStamped.html>`_

    :rtype: list of strings
    '''
    return _get_topic(masteruri, 'LinkStatesStamped', wait, check_host)


def _get_topic(masteruri, ttype, wait=True, check_host=True):
    result = []
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('interface_finder_get_topic_node')
    try:
        while not result and rclpy.ok():
            topic_names_and_types = node.get_topic_names_and_types()
            for topic_name, topic_types in topic_names_and_types:
                for t in topic_types:
                    if ttype in t:
                        result.append(topic_name)
                        break
            if not result and wait:
                Log.warn(f'Wait for topic type "{ttype}".')
                time.sleep(1)
            if not wait:
                break
    finally:
        node.destroy_node()
    return result


def get_listmaster_service(masteruri, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    `list_masters` and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the service

    :type wait: boo

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `list_masters`

    :rtype: list of strings
    '''
    return _get_service(masteruri, 'list_masters', wait, check_host)


def get_refresh_service(masteruri, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    `refresh` and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the service

    :type wait: boo

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `refresh`

    :rtype: list of strings
    '''
    return _get_service(masteruri, 'refresh', wait, check_host)


def _get_service(masteruri, name, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    given name and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param name: the ending name of the service

    :type name: str

    :param wait: check every second for the service

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `refresh`

    :rtype: list of strings
    '''
    result = []
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('interface_finder_get_service_node')
    try:
        while not result and rclpy.ok():
            service_names_and_types = node.get_service_names_and_types()
            for srv_name, srv_types in service_names_and_types:
                if srv_name.endswith(name):
                    result.append(srv_name)
            if not result and wait:
                Log.warn(f'Wait for service "{name}".')
                time.sleep(1)
            if not wait:
                break
    finally:
        node.destroy_node()
    return result