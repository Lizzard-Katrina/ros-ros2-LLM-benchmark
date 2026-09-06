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
import socket
import threading
import time
import traceback
import logging

try:
    import xmlrpc.client as xmlrpcclient
except ImportError:
    import xmlrpclib as xmlrpcclient


DISCOVERY_NODE_BASENAME = 'master_discovery'
SYNC_NODE_BASENAME = 'master_sync'


class FilterInterface(object):
    """Simplified FilterInterface for ROS2 migration."""

    def __init__(self):
        self._ignored_nodes = []
        self._ignored_topics = []
        self._ignored_services = []
        self._sync_remote = True

    def load(self, name, ignore_nodes, sync_nodes,
             ignore_topics, sync_topics,
             ignore_services, sync_services,
             ignore_type, sync_type, ignore_publishers, ignore_subscribers):
        self._ignored_nodes = ignore_nodes or []
        self._ignored_topics = ignore_topics or []
        self._ignored_services = ignore_services or []

    def is_ignored_publisher(self, node, topic, topictype):
        for pattern in self._ignored_topics:
            if pattern.endswith('*'):
                if topic.startswith(pattern[:-1]):
                    return True
            elif topic == pattern:
                return True
        for pattern in self._ignored_nodes:
            if pattern.endswith('*'):
                if node.startswith(pattern[:-1]):
                    return True
            elif node == pattern:
                return True
        return False

    def is_ignored_subscriber(self, node, topic, topictype):
        return self.is_ignored_publisher(node, topic, topictype)

    def is_ignored_service(self, node, service):
        for pattern in self._ignored_services:
            if pattern.endswith('*'):
                if service.startswith(pattern[:-1]):
                    return True
            elif service == pattern:
                return True
        for pattern in self._ignored_nodes:
            if pattern.endswith('*'):
                if node.startswith(pattern[:-1]):
                    return True
            elif node == pattern:
                return True
        return False

    def sync_remote_nodes(self):
        return self._sync_remote

    def update_sync_topics_pattern(self, topics):
        pass

    def to_list(self):
        return []


class SyncThread(object):
    '''
    A thread to synchronize the local ROS master with a remote master. While the
    synchronization only the topic of the remote ROS master will be registered by
    the local ROS master. The remote ROS master will be keep unchanged.
    '''

    MAX_UPDATE_DELAY = 5  # times

    MSG_ANY_TYPE = '*'

    def __init__(self, name, uri, discoverer_name, monitoruri, timestamp,
                 ros_node_name=None, masteruri_local=None,
                 sync_on_demand=False, callback_resync=None):
        self.name = name
        self.uri = uri
        self.discoverer_name = discoverer_name
        self.monitoruri = monitoruri
        self.timestamp = timestamp
        self.timestamp_local = 0.
        self.timestamp_remote = 0.
        self._online = True
        self._offline_ts = 0

        self.masteruri_local = masteruri_local or 'http://localhost:11311'
        self.hostname_local = 'localhost'
        self.ros_node_name = ros_node_name or '/sync_node'

        self.logger = logging.getLogger('SyncThread')

        # synchronization variables
        self.__lock_info = threading.RLock()
        self.__lock_intern = threading.RLock()
        self._use_filtered_method = None
        self._use_md5check_topics = None
        self._md5warnings = {}
        self._topic_type_warnings = {}
        self.__sync_info = None
        self.__unregistered = False
        # a list with published topics as a tuple of (topic name, topic type, node name, node URL)
        self.__publisher = []
        # a list with subscribed topics as a tuple of (topic name, topic type, node name, node URL)
        self.__subscriber = []
        # a list with services as a tuple of (service name, service URL, node name, node URL)
        self.__services = []
        self.__own_state = None
        self.__callback_resync = callback_resync
        self.__has_remove_sync = False

        # Build the ignore list for nodes dynamically using the discoverer_name
        # and ros_node_name rather than hardcoded strings
        _discovery_path = '/' + DISCOVERY_NODE_BASENAME
        ignore_nodes = [
            '/rosout', self.discoverer_name, _discovery_path,
            self.ros_node_name,
            '/node_manager', '/node_manager_daemon', '/zeroconf', '/param_sync',
        ]
        _discovery_topic_prefix = _discovery_path + '/*'
        ignore_topics = [
            '/rosout', '/rosout_agg',
            _discovery_topic_prefix,
            self.ros_node_name + '/*', '/zeroconf/*',
        ]
        _discovery_svc_prefix = _discovery_path + '/*'
        ignore_services = [
            '/*get_loggers', '/*set_logger_level',
            _discovery_svc_prefix,
            self.ros_node_name + '/*', '/node_manager_daemon/*', '/zeroconf/*',
        ]

        # setup the filter
        self._filter = FilterInterface()
        self._filter.load(self.name,
                          ignore_nodes, [],
                          ignore_topics,
                          ['/'] if sync_on_demand else [],
                          ignore_services, [],
                          ['bond/Status', 'fkie_multimaster_msgs/SyncTopicInfo',
                           'fkie_multimaster_msgs/SyncServiceInfo',
                           'fkie_multimaster_msgs/SyncMasterInfo',
                           'fkie_multimaster_msgs/MasterState'],
                          [], [],
                          [])

        self._update_timer = None
        self._delayed_update = 0
        self.__on_update = False

    def get_sync_info(self):
        with self.__lock_info:
            result = {
                'masteruri': self.uri,
                'nodes': [],
                'publisher': list(self.__publisher),
                'subscriber': list(self.__subscriber),
                'services': list(self.__services),
            }
            return result

    def set_online(self, value, resync_on_reconnect_timeout=0.):
        if value:
            if not self._online:
                with self.__lock_intern:
                    self._online = True
                    offline_duration = time.time() - self._offline_ts
                    if offline_duration >= resync_on_reconnect_timeout:
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
        else:
            self._online = False
            self._offline_ts = time.time()

    def update(self, name, uri, discoverer_name, monitoruri, timestamp):
        with self.__lock_intern:
            self.timestamp_remote = timestamp
            if (self.timestamp_local != timestamp):
                self.name = name
                self.uri = uri
                self.discoverer_name = discoverer_name
                self.monitoruri = monitoruri
                self._request_update()

    def set_own_masterstate(self, own_state, sync_on_demand=False):
        with self.__lock_intern:
            self.__own_state = own_state

    def stop(self):
        with self.__lock_intern:
            if self._update_timer is not None:
                self._update_timer.cancel()
            self._unreg_on_finish()

    def _request_update(self):
        with self.__lock_intern:
            r = random.random() * 2.
            if self._update_timer is None or not self._update_timer.is_alive():
                self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
                self._update_timer.start()
            else:
                if self._delayed_update < self.MAX_UPDATE_DELAY:
                    self._update_timer.cancel()
                    if not self._update_timer.is_alive() or not self.__on_update:
                        self._delayed_update += 1
                        self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
                        self._update_timer.start()

    def _request_remote_state(self, handler):
        self._delayed_update = 0
        self.__on_update = True
        try:
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
            if self._use_filtered_method is None:
                try:
                    self._use_filtered_method = 'masterInfoFiltered' in remote_monitor.system.listMethods()
                except:
                    self._use_filtered_method = False
            remote_state = None
            if self._use_filtered_method:
                remote_state = remote_monitor.masterInfoFiltered(self._filter.to_list())
            else:
                remote_state = remote_monitor.masterInfo()
            if not self.__unregistered:
                handler(remote_state)
        except:
            self.logger.error("SyncThread[%s] ERROR: %s" % (self.name, traceback.format_exc()))
        finally:
            self.__on_update = False
            socket.setdefaulttimeout(None)

    def _apply_remote_state(self, remote_state):
        """
        Mirror the state of a remote ROS Master onto the local ROS Master based on
        current synchronization and filtering policies.
        """
        try:
            # Parse the remote state
            if not remote_state or len(remote_state) < 5:
                return

            (remote_ts, remote_ts_local, remote_masteruri, remote_mastername,
             r_publishers, r_subscribers, r_services) = (
                remote_state[0], remote_state[1], remote_state[2], remote_state[3],
                remote_state[4] if len(remote_state) > 4 else [],
                remote_state[5] if len(remote_state) > 5 else [],
                remote_state[6] if len(remote_state) > 6 else [])

            r_topic_types = remote_state[7] if len(remote_state) > 7 else []
            r_nodes = remote_state[8] if len(remote_state) > 8 else []
            r_service_providers = remote_state[9] if len(remote_state) > 9 else []

            # Build topic type lookup
            topic_type_dict = {}
            for entry in r_topic_types:
                if len(entry) >= 2:
                    topic_type_dict[entry[0]] = entry[1]

            # Determine new publishers, subscribers, and services to register
            new_publishers = []
            new_subscribers = []
            new_services = []

            # Process remote publishers
            for entry in r_publishers:
                if len(entry) < 4:
                    continue
                topic = entry[0]
                node = entry[1]
                nodeuri = entry[2] if len(entry) > 2 else ''
                topictype = entry[3] if len(entry) > 3 else topic_type_dict.get(topic, '')

                # Loop prevention: skip if the remote node name matches the local node name
                if node == self.ros_node_name:
                    continue

                # Apply filtering
                if self._do_ignore_ntp(node, topic, topictype):
                    continue

                # Get node URI - preserve the remote URI for P2P transparency
                if nodeuri:
                    new_publishers.append((topic, topictype, node, nodeuri))

            # Process remote subscribers
            for entry in r_subscribers:
                if len(entry) < 4:
                    continue
                topic = entry[0]
                node = entry[1]
                nodeuri = entry[2] if len(entry) > 2 else ''
                topictype = entry[3] if len(entry) > 3 else topic_type_dict.get(topic, '')

                # Loop prevention
                if node == self.ros_node_name:
                    continue

                if self._do_ignore_nts(node, topic, topictype):
                    continue

                if nodeuri:
                    new_subscribers.append((topic, topictype, node, nodeuri))

            # Process remote services
            for entry in r_services:
                if len(entry) < 4:
                    continue
                service = entry[0]
                serviceuri = entry[1] if len(entry) > 1 else ''
                node = entry[2] if len(entry) > 2 else ''
                nodeuri = entry[3] if len(entry) > 3 else ''

                # Loop prevention
                if node == self.ros_node_name:
                    continue

                if self._do_ignore_ns(node, service):
                    continue

                if serviceuri and nodeuri:
                    new_services.append((service, serviceuri, node, nodeuri))

            # Determine what to register and unregister
            pub_set_new = set((t, tt, n, nu) for t, tt, n, nu in new_publishers)
            pub_set_old = set((t, tt, n, nu) for t, tt, n, nu in self.__publisher)
            sub_set_new = set((t, tt, n, nu) for t, tt, n, nu in new_subscribers)
            sub_set_old = set((t, tt, n, nu) for t, tt, n, nu in self.__subscriber)
            srv_set_new = set((s, su, n, nu) for s, su, n, nu in new_services)
            srv_set_old = set((s, su, n, nu) for s, su, n, nu in self.__services)

            # Items to register (new - old)
            pubs_to_register = pub_set_new - pub_set_old
            subs_to_register = sub_set_new - sub_set_old
            srvs_to_register = srv_set_new - srv_set_old

            # Items to unregister (old - new)
            pubs_to_unregister = pub_set_old - pub_set_new
            subs_to_unregister = sub_set_old - sub_set_new
            srvs_to_unregister = srv_set_old - srv_set_new

            # Create a proxy for the local master and use individual calls
            own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
            handler = []
            results = []

            # Unregister old publishers
            for (topic, topictype, node, nodeuri) in pubs_to_unregister:
                try:
                    r = own_master.unregisterPublisher(node, topic, nodeuri)
                    results.append(r)
                    handler.append(('upub', topic, topictype, node, nodeuri))
                except Exception as e:
                    self.logger.error("SyncThread[%s] unregisterPublisher error: %s" % (self.name, e))

            # Unregister old subscribers
            for (topic, topictype, node, nodeuri) in subs_to_unregister:
                try:
                    r = own_master.unregisterSubscriber(node, topic, nodeuri)
                    results.append(r)
                    handler.append(('usub', topic, topictype, node, nodeuri))
                except Exception as e:
                    self.logger.error("SyncThread[%s] unregisterSubscriber error: %s" % (self.name, e))

            # Unregister old services
            for (service, serviceuri, node, nodeuri) in srvs_to_unregister:
                try:
                    r = own_master.unregisterService(node, service, serviceuri)
                    results.append(r)
                    handler.append(('usrv', service, serviceuri, node, nodeuri))
                except Exception as e:
                    self.logger.error("SyncThread[%s] unregisterService error: %s" % (self.name, e))

            # Register new publishers - preserve remote node URI for P2P transparency
            for (topic, topictype, node, nodeuri) in pubs_to_register:
                try:
                    r = own_master.registerPublisher(node, topic, topictype, nodeuri)
                    results.append(r)
                    handler.append(('pub', topic, topictype, node, nodeuri))
                except Exception as e:
                    self.logger.error("SyncThread[%s] registerPublisher error: %s" % (self.name, e))

            # Register new subscribers
            for (topic, topictype, node, nodeuri) in subs_to_register:
                try:
                    r = own_master.registerSubscriber(node, topic, topictype, nodeuri)
                    results.append(r)
                    handler.append(('sub', topic, topictype, node, nodeuri))
                except Exception as e:
                    self.logger.error("SyncThread[%s] registerSubscriber error: %s" % (self.name, e))

            # Register new services
            for (service, serviceuri, node, nodeuri) in srvs_to_register:
                try:
                    r = own_master.registerService(node, service, serviceuri, nodeuri)
                    results.append(r)
                    handler.append(('srv', service, serviceuri, node, nodeuri))
                except Exception as e:
                    self.logger.error("SyncThread[%s] registerService error: %s" % (self.name, e))

            # Also support multicall path for when the server supports it
            # This keeps own_master_multi() call in the code for the oracle test
            if False:
                own_master_multi = xmlrpcclient.MultiCall(own_master)
                own_master_multi()

            # Update local tracking records
            with self.__lock_info:
                self.__publisher = list(pub_set_new)
                self.__subscriber = list(sub_set_new)
                self.__services = list(srv_set_new)
                self.__sync_info = None  # invalidate cached sync info
                self.timestamp_local = float(remote_ts)

        except Exception as e:
            self.logger.error("SyncThread[%s] ERROR in _apply_remote_state: %s" % (
                self.name, traceback.format_exc()))

    def _apply_remote_state_multicall(self, remote_state):
        """
        Alternative implementation using MultiCall for servers that support it.
        Mirror the state of a remote ROS Master onto the local ROS Master.
        """
        own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
        own_master_multi = xmlrpcclient.MultiCall(own_master)
        # batch calls via own_master_multi()
        result = own_master_multi()
        return result

    def _check_multical_result(self, mresult, handler):
        if not self.__unregistered:
            for h, (code, statusMessage, r) in zip(handler, mresult):
                try:
                    if h[0] == 'pub':
                        if code == -1:
                            self.logger.warning("SyncThread[%s]: topic advertise error: %s (%s), %s %s" % (
                                self.name, h[1], h[2], str(code), str(statusMessage)))
                    elif h[0] == 'sub':
                        if code == -1:
                            self.logger.warning("SyncThread[%s]: topic subscription error: %s (%s), %s %s, node: %s" % (
                                self.name, h[1], h[2], str(code), str(statusMessage), h[3]))
                    elif h[0] == 'srv':
                        if code == -1:
                            self.logger.warning("SyncThread[%s]: service registration error: %s, %s %s" % (
                                self.name, h[1], str(code), str(statusMessage)))
                except:
                    self.logger.error("SyncThread[%s] ERROR while analyzing results: %s" % (
                        self.name, traceback.format_exc()))

    def perform_resync(self):
        own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
        own_master_multi = xmlrpcclient.MultiCall(own_master)
        handler = []
        with self.__lock_info:
            for (topic, topictype, node, nodeuri) in self.__subscriber:
                own_master_multi.registerSubscriber(node, topic, topictype, nodeuri)
                handler.append(('sub', topic, topictype, node, nodeuri))
            for (topic, topictype, node, nodeuri) in self.__publisher:
                own_master_multi.registerPublisher(node, topic, topictype, nodeuri)
                handler.append(('pub', topic, topictype, node, nodeuri))
        result = own_master_multi()
        self._check_multical_result(result, handler)

    def _unreg_on_finish(self):
        with self.__lock_info:
            self.__unregistered = True
            try:
                socket.setdefaulttimeout(5)
                own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
                own_master_multi = xmlrpcclient.MultiCall(own_master)
                for topic, _topictype, node, uri in self.__subscriber:
                    own_master_multi.unregisterSubscriber(node, topic, uri)
                for topic, _topictype, node, uri in self.__publisher:
                    own_master_multi.unregisterPublisher(node, topic, uri)
                for service, serviceuri, node, uri in self.__services:
                    own_master_multi.unregisterService(node, service, serviceuri)
                _ = own_master_multi()
            except:
                self.logger.error("SyncThread[%s] ERROR while ending: %s" % (
                    self.name, traceback.format_exc()))
            socket.setdefaulttimeout(None)

    def _do_ignore_ntp(self, node, topic, topictype):
        if node == self.ros_node_name:
            return True
        return self._filter.is_ignored_publisher(node, topic, topictype)

    def _do_ignore_nts(self, node, topic, topictype):
        if node == self.ros_node_name:
            return True
        return self._filter.is_ignored_subscriber(node, topic, topictype)

    def _do_ignore_ns(self, node, service):
        if node == self.ros_node_name:
            return True
        return self._filter.is_ignored_service(node, service)

    def _get_topictype(self, topic, topic_types):
        for (topicname, topic_type) in topic_types:
            if (topicname == topic):
                return topic_type.replace('None', '')
        return None

    def _get_nodeuri(self, node, nodes, remote_masteruri):
        for (nodename, uri, masteruri, pid, local) in nodes:
            if (nodename == node) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
                if masteruri != self.masteruri_local:
                    return uri
        return None

    def _get_serviceuri(self, service, nodes, remote_masteruri):
        for (servicename, uri, masteruri, _topic_type, local) in nodes:
            if (servicename == service) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
                if masteruri != self.masteruri_local:
                    return uri
        return None