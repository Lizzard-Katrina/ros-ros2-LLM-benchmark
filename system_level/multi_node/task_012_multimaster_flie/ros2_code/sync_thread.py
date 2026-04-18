# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



import random
import roslib
import roslib.message
import socket
import threading
import time
import traceback
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

from fkie_multimaster_msgs.msg import SyncTopicInfo, SyncServiceInfo, SyncMasterInfo
import rclpy
from rclpy.node import Node
from fkie_master_discovery.common import masteruri_from_ros, get_hostname
from fkie_master_discovery.filter_interface import FilterInterface


class SyncThread(object):
    '''
    A thread to synchronize the local ROS master with a remote master. While the
    synchronization only the topic of the remote ROS master will be registered by
    the local ROS master. The remote ROS master will be keep unchanged.
    '''

    MAX_UPDATE_DELAY = 5  # times

    MSG_ANY_TYPE = '*'

    def __init__(self, name, uri, discoverer_name, monitoruri, timestamp, sync_on_demand=False, callback_resync=None):
        '''
        Initialization method for the SyncThread.
        @param name: the name of the ROS master synchronized with.
        @type name:  C{str}
        @param uri: the URI of the ROS master synchronized with
        @type uri:  C{str}
        @param discoverer_name: the name of the discovery node running on ROS master synchronized with.
        @type discoverer_name:  C{str}
        @param monitoruri: The URI of RPC server of the discovery node to get the ROS master state by calling a method only once.
        @type monitoruri:  C{str}
        @param timestamp: The timestamp of the current state of the ROS master info.
        @type timestamp:  C{float64}
        @param sync_on_demand: Synchronize topics on demand
        @type sync_on_demand: bool
        '''
        self.name = name
        self.uri = uri
        self.discoverer_name = discoverer_name
        self.monitoruri = monitoruri
        self.timestamp = timestamp
        self.timestamp_local = 0.
        self.timestamp_remote = 0.
        self._online = True
        self._offline_ts = 0

        self.masteruri_local = masteruri_from_ros()
        self.hostname_local = get_hostname(self.masteruri_local)
        print("SyncThread[%s]: create this sync thread, discoverer_name: %s" % (self.name, self.discoverer_name))
        # synchronization variables
        self.__lock_info = threading.RLock()
        self.__lock_intern = threading.RLock()
        self._use_filtered_method = None
        self._use_md5check_topics = None
        self._md5warnings = {}  # ditionary of {(topicname, node, nodeuri) : (topictype, md5sum)}
        self._topic_type_warnings = {}  # ditionary of {(topicname, node, nodeuri) : remote topictype}
        # SyncMasterInfo with currently synchronized nodes, publisher (topic, node, nodeuri), subscriber(topic, node, nodeuri) and services
        self.__sync_info = None
        self.__unregistered = False
        # a list with published topics as a tuple of (topic name, node name, node URL)
        self.__publisher = []
        # a list with subscribed topics as a tuple of (topic name, node name, node URL)
        self.__subscriber = []
        # a list with services as a tuple of (service name, service URL, node name, node URL)
        self.__services = []
        # the state of the own ROS master is used if `sync_on_demand` is enabled or
        # to determine the type of topic subscribed remote with `Empty` type
        self.__own_state = None
        self.__callback_resync = callback_resync
        self.__has_remove_sync = False

        # setup the filter
        self._filter = FilterInterface()
        self._filter.load(self.name,
                          ['/rosout', self.discoverer_name, '/master_discovery', '/master_sync', '/node_manager', '/node_manager_daemon', '/zeroconf', '/param_sync'], [],
                          ['/rosout', '/rosout_agg', '/master_discovery/*', '/master_sync/*', '/zeroconf/*'], ['/'] if sync_on_demand else [],
                          ['/*get_loggers', '/*set_logger_level', '/master_discovery/*', '/master_sync/*', '/node_manager_daemon/*', '/zeroconf/*'], [],
                          # do not sync the bond message of the nodelets!!
                          ['bond/Status', 'fkie_multimaster_msgs/SyncTopicInfo', 'fkie_multimaster_msgs/SyncServiceInfo', 'fkie_multimaster_msgs/SyncMasterInfo', 'fkie_multimaster_msgs/MasterState'],
                          [], [],
                          [])

        # congestion avoidance: wait for random.random*2 sec. If an update request
        # is received try to cancel and restart the current timer. The timer can be
        # canceled for maximal MAX_UPDATE_DELAY times.
        self._update_timer = None
        self._delayed_update = 0
        self.__on_update = False

    def get_sync_info(self):
        '''
        Returns the synchronized publisher, subscriber and services.
        @rtype: SyncMasterInfo
        '''
        with self.__lock_info:
            if self.__sync_info is None:
                # create a sync info
                result_set = set()
                result_publisher = []
                result_subscriber = []
                result_services = []
                for (t_n, _t_t, n_n, n_uri) in self.__publisher:
                    result_publisher.append(SyncTopicInfo(t_n, n_n, n_uri))
                    result_set.add(n_n)
                for (t_n, _t_t, n_n, n_uri) in self.__subscriber:
                    result_subscriber.append(SyncTopicInfo(t_n, n_n, n_uri))
                    result_set.add(n_n)
                for (s_n, s_uri, n_n, n_uri) in self.__services:
                    result_services.append(SyncServiceInfo(s_n, s_uri, n_n, n_uri))
                    result_set.add(n_n)
                self.__sync_info = SyncMasterInfo(self.uri, list(result_set), result_publisher, result_subscriber, result_services)
            return self.__sync_info

    def set_online(self, value, resync_on_reconnect_timeout=0.):
        if value:
            if not self._online:
                with self.__lock_intern:
                    self._online = True
                    offline_duration = time.time() - self._offline_ts
                    if offline_duration >= resync_on_reconnect_timeout:
                        print("SyncThread[%s]: perform resync after the host was offline (unregister and register again to avoid connection losses to python topic. These does not suppot reconnection!)" % self.name)
                        if self._update_timer is not None:
                            self._update_timer.cancel()
                        self._unreg_on_finish()
                        self.__unregistered = False
                        self.__publisher = []
                        self.__subscriber = []
                        self.__services = []
                        self.timestamp = 0.
                        self.timestamp_local = 0.
                        self.timestamp_remote = 0.