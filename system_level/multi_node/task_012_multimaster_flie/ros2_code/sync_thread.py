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
try:
    import xmlrpc.client as xmlrpcclient
except ImportError:
    import xmlrpclib as xmlrpcclient

from fkie_multimaster_msgs.msg import SyncTopicInfo, SyncServiceInfo, SyncMasterInfo
import rclpy

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

    def __init__(self, name, uri, discoverer_name, monitoruri, timestamp, sync_on_demand=False, callback_resync=None, node=None):
        self.node = node
        self.name = name
        self.uri = uri
        self.discoverer_name = discoverer_name
        self.monitoruri = monitoruri
        self.timestamp = timestamp
        self.timestamp_local = 0.
        self.timestamp_remote = 0.
        self._online = True
        self._offline_ts = 0
        self.ros_node_name = str(self.node.get_name()) if self.node else 'sync_thread'

        self.masteruri_local = masteruri_from_ros()
        self.hostname_local = get_hostname(self.masteruri_local)
        if self.node:
            self.node.get_logger().debug("SyncThread[%s]: create this sync thread, discoverer_name: %s" % (self.name, self.discoverer_name))
        self.__lock_info = threading.RLock()
        self.__lock_intern = threading.RLock()
        self._use_filtered_method = None
        self._use_md5check_topics = None
        self._md5warnings = {}
        self._topic_type_warnings = {}
        self.__sync_info = None
        self.__unregistered = False
        self.__publisher = []
        self.__subscriber = []
        self.__services = []
        self.__own_state = None
        self.__callback_resync = callback_resync
        self.__has_remove_sync = False

        self._filter = FilterInterface()
        self._filter.load(self.name,
                          ['/rosout', self.discoverer_name, '/master_discovery', '/master_sync', '/node_manager', '/node_manager_daemon', '/zeroconf', '/param_sync'], [],
                          ['/rosout', '/rosout_agg', '/master_discovery/*', '/master_sync/*', '/zeroconf/*'], ['/'] if sync_on_demand else [],
                          ['/*get_loggers', '/*set_logger_level', '/master_discovery/*', '/master_sync/*', '/node_manager_daemon/*', '/zeroconf/*'], [],
                          ['bond/Status', 'fkie_multimaster_msgs/SyncTopicInfo', 'fkie_multimaster_msgs/SyncServiceInfo', 'fkie_multimaster_msgs/SyncMasterInfo', 'fkie_multimaster_msgs/MasterState'],
                          [], [],
                          [])

        self._update_timer = None
        self._delayed_update = 0
        self.__on_update = False

    def get_sync_info(self):
        with self.__lock_info:
            if self.__sync_info is None:
                result_set = set()
                result_publisher = []
                result_subscriber = []
                result_services = []
                for (t_n, _t_t, n_n, n_uri) in self.__publisher:
                    result_publisher.append(SyncTopicInfo(topic=t_n, node=n_n, nodeuri=n_uri))
                    result_set.add(n_n)
                for (t_n, _t_t, n_n, n_uri) in self.__subscriber:
                    result_subscriber.append(SyncTopicInfo(topic=t_n, node=n_n, nodeuri=n_uri))
                    result_set.add(n_n)
                for (s_n, s_uri, n_n, n_uri) in self.__services:
                    result_services.append(SyncServiceInfo(service=s_n, serviceuri=s_uri, node=n_n, nodeuri=n_uri))
                    result_set.add(n_n)
                self.__sync_info = SyncMasterInfo(masteruri=self.uri, nodes=list(result_set), publisher=result_publisher, subscriber=result_subscriber, services=result_services)
            return self.__sync_info

    def set_online(self, value, resync_on_reconnect_timeout=0.):
        if value:
            if not self._online:
                with self.__lock_intern:
                    self._online = True
                    offline_duration = time.time() - self._offline_ts
                    if offline_duration >= resync_on_reconnect_timeout:
                        if self.node:
                            self.node.get_logger().info("SyncThread[%s]: perform resync after the host was offline" % self.name)
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
                        if self.node:
                            self.node.get_logger().info("SyncThread[%s]: skip resync after the host was offline" % self.name)
        else:
            self._online = False
            self._offline_ts = time.time()

    def update(self, name, uri, discoverer_name, monitoruri, timestamp):
        with self.__lock_intern:
            self.timestamp_remote = timestamp
            if (self.timestamp_local != timestamp):
                if self.node:
                    self.node.get_logger().debug("SyncThread[%s]: update notify new timestamp(%.9f), old(%.9f)" % (self.name, timestamp, self.timestamp_local))
                self.name = name
                self.uri = uri
                self.discoverer_name = discoverer_name
                self.monitoruri = monitoruri
                self._request_update()

    def set_own_masterstate(self, own_state, sync_on_demand=False):
        with self.__lock_intern:
            timestamp_local = own_state.timestamp_local
            if self.__own_state is None or (self.__own_state.timestamp_local != timestamp_local):
                ownstate_ts = self.__own_state.timestamp_local if self.__own_state is not None else float('nan')
                if self.node:
                    self.node.get_logger().debug("SyncThread[%s]: local state update notify new timestamp(%.9f), old(%.9f)" % (self.name, timestamp_local, ownstate_ts))
                self.__own_state = own_state
                if sync_on_demand:
                    self._filter.update_sync_topics_pattern(self.__own_state.topic_names)
                self._request_update()

    def stop(self):
        if self.node:
            self.node.get_logger().debug("  SyncThread[%s]: stop request" % self.name)
        with self.__lock_intern:
            if self._update_timer is not None:
                self._update_timer.cancel()
            self._unreg_on_finish()
        if self.node:
            self.node.get_logger().debug("  SyncThread[%s]: stop exit" % self.name)

    def _request_update(self):
        with self.__lock_intern:
            r = random.random() * 2.
            if self._update_timer is None or not self._update_timer.is_alive():
                del self._update_timer
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
            if self.node:
                self.node.get_logger().info("SyncThread[%s] Requesting remote state from '%s'" % (self.name, self.monitoruri))
            if self._use_filtered_method:
                remote_state = remote_monitor.masterInfoFiltered(self._filter.to_list())
            else:
                remote_state = remote_monitor.masterInfo()
            if not self.__unregistered:
                handler(remote_state)
        except:
            if self.node:
                self.node.get_logger().error("SyncThread[%s] ERROR: %s" % (self.name, traceback.format_exc()))
        finally:
            self.__on_update = False
            socket.setdefaulttimeout(None)

    def _apply_remote_state(self, remote_state):
        own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
        own_master_multi = xmlrpcclient.MultiCall(own_master)
        handler = []
        
        with self.__lock_info:
            stamp, masteruri, name, nodename, nodeuri, publishers, subscribers, services, topic_types, nodes, service_providers = remote_state
            
            for topic, nodes_list in publishers:
                ttype = self._get_topictype(topic, topic_types)
                for node in nodes_list:
                    if node == self.ros_node_name or self._do_ignore_ntp(node, topic, ttype):
                        continue
                    nuri = self._get_nodeuri(node, nodes, masteruri)
                    if nuri:
                        own_master_multi.registerPublisher(node, topic, ttype, nuri)
                        handler.append(('pub', topic, ttype, node, nuri))
                        self.__publisher.append((topic, ttype, node, nuri))
                        
            for topic, nodes_list in subscribers:
                ttype = self._get_topictype(topic, topic_types)
                for node in nodes_list:
                    if node == self.ros_node_name or self._do_ignore_nts(node, topic, ttype):
                        continue
                    nuri = self._get_nodeuri(node, nodes, masteruri)
                    if nuri:
                        own_master_multi.registerSubscriber(node, topic, ttype, nuri)
                        handler.append(('sub', topic, ttype, node, nuri))
                        self.__subscriber.append((topic, ttype, node, nuri))
                        
            for service, nodes_list in services:
                for node in nodes_list:
                    if node == self.ros_node_name or self._do_ignore_ns(node, service):
                        continue
                    suri = self._get_serviceuri(service, service_providers, masteruri)
                    if suri:
                        own_master_multi.registerService(node, service, suri, suri)
                        handler.append(('srv', service, suri, node, suri))
                        self.__services.append((service, suri, node, suri))
                        
        result = own_master_multi()
        self._check_multical_result(result, handler)

    def _check_multical_result(self, mresult, handler):
        if not self.__unregistered:
            publiser_to_update = {}
            for h, (code, statusMessage, r) in zip(handler, mresult):
                try:
                    if h[0] == 'sub':
                        if code == -1:
                            if self.node:
                                self.node.get_logger().warn("SyncThread[%s]: topic subscription error: %s (%s), %s %s, node: %s" % (self.name, h[1], h[2], str(code), str(statusMessage), h[3]))
                        else:
                            if self.node:
                                self.node.get_logger().debug("SyncThread[%s]: topic subscribed: %s, %s %s, node: %s" % (self.name, h[1], str(code), str(statusMessage), h[3]))
                    if h[0] == 'sub' and code == 1 and len(r) > 0:
                        if not self._do_ignore_ntp(h[3], h[1], h[2]):
                            publiser_to_update[(h[1], h[4], h[3])] = r
                    elif h[0] == 'pub':
                        if code == -1:
                            if self.node:
                                self.node.get_logger().warn("SyncThread[%s]: topic advertise error: %s (%s), %s %s" % (self.name, h[1], h[2], str(code), str(statusMessage)))
                        else:
                            if self.node:
                                self.node.get_logger().debug("SyncThread[%s]: topic advertised: %s, %s %s" % (self.name, h[1], str(code), str(statusMessage)))
                    elif h[0] == 'usub':
                        if self.node:
                            self.node.get_logger().debug("SyncThread[%s]: topic unsubscribed: %s, %s %s" % (self.name, h[1], str(code), str(statusMessage)))
                    elif h[0] == 'upub':
                        if self.node:
                            self.node.get_logger().debug("SyncThread[%s]: topic unadvertised: %s, %s %s" % (self.name, h[1], str(code), str(statusMessage)))
                    elif h[0] == 'srv':
                        if code == -1:
                            if self.node:
                                self.node.get_logger().warn("SyncThread[%s]: service registration error: %s, %s %s" % (self.name, h[1], str(code), str(statusMessage)))
                        else:
                            if self.node:
                                self.node.get_logger().debug("SyncThread[%s]: service registered: %s, %s %s" % (self.name, h[1], str(code), str(statusMessage)))
                    elif h[0] == 'usrv':
                        if self.node:
                            self.node.get_logger().debug("SyncThread[%s]: service unregistered: %s, %s %s" % (self.name, h[1], str(code), str(statusMessage)))
                except:
                    if self.node:
                        self.node.get_logger().error("SyncThread[%s] ERROR while analyzing the results of the registration call [%s]: %s" % (self.name, h[1], traceback.format_exc()))
            for (sub_topic, api, node), pub_uris in publiser_to_update.items():
                msg = "SyncThread[%s] publisherUpdate[%s] -> node: %s [%s], publisher uris: %s" % (self.name, sub_topic, api, node, pub_uris)
                try:
                    pub_client = xmlrpcclient.ServerProxy(api)
                    ret = pub_client.publisherUpdate('/master', sub_topic, pub_uris)
                    msg_suffix = "result=%s" % ret
                    if self.node:
                        self.node.get_logger().debug("%s: %s" % (msg, msg_suffix))
                except Exception as ex:
                    msg_suffix = "exception=%s" % ex
                    if self.node:
                        self.node.get_logger().warn("%s: %s" % (msg, msg_suffix))

    def perform_resync(self):
        own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
        own_master_multi = xmlrpcclient.MultiCall(own_master)
        handler = []
        with self.__lock_info:
            for (topic, topictype, node, nodeuri) in self.__subscriber:
                own_master_multi.registerSubscriber(node, topic, topictype, nodeuri)
                if self.node:
                    self.node.get_logger().debug("SyncThread[%s]: prepare RESUB %s[%s] %s[%s]" % (self.name, node, nodeuri, topic, topictype))
                handler.append(('sub', topic, topictype, node, nodeuri))
            for (topic, topictype, node, nodeuri) in self.__publisher:
                own_master_multi.registerPublisher(node, topic, topictype, nodeuri)
                if self.node:
                    self.node.get_logger().debug("SyncThread[%s]: prepare REPUB %s[%s] %s[%s]" % (self.name, node, nodeuri, topic, topictype))
                handler.append(('pub', topic, topictype, node, nodeuri))
        result = own_master_multi()
        self._check_multical_result(result, handler)

    def _check_md5sums(self, topics_to_register):
        try:
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
            if self._use_md5check_topics is None:
                try:
                    self._use_md5check_topics = 'getTopicsMd5sum' in remote_monitor.system.listMethods()
                except:
                    self._use_md5check_topics = False
            if self._use_md5check_topics:
                if self.node:
                    self.node.get_logger().info("SyncThread[%s] Requesting remote md5sums '%s'" % (self.name, self.monitoruri))
                topic_types = [topictype for _topic, topictype, _node, _nodeuri in topics_to_register]
                remote_md5sums_topics = remote_monitor.getTopicsMd5sum(topic_types)
                for rttype, rtmd5sum in remote_md5sums_topics:
                    try:
                        lmd5sum = None
                        if lmd5sum != rtmd5sum:
                            for topicname, topictype, node, nodeuri in topics_to_register:
                                if topictype == rttype:
                                    if (topicname, node, nodeuri) not in self._md5warnings:
                                        if lmd5sum is None:
                                            if self.node:
                                                self.node.get_logger().warn("Unknown message type %s for topic: %s, local host: %s, remote host: %s" % (rttype, topicname, self.hostname_local, self.name))
                                        else:
                                            if self.node:
                                                self.node.get_logger().warn("Different checksum detected for topic: %s, type: %s, local host: %s, remote host: %s" % (topicname, rttype, self.hostname_local, self.name))
                                        self._md5warnings[(topicname, node, nodeuri)] = (topictype, lmd5sum)
                    except Exception as err:
                        import traceback
                        if self.node:
                            self.node.get_logger().warn(str(err))
                            self.node.get_logger().warn(traceback.format_exc())
        except:
            import traceback
            if self.node:
                self.node.get_logger().error("SyncThread[%s] ERROR: %s" % (self.name, traceback.format_exc()))
        finally:
            socket.setdefaulttimeout(None)

    def _check_local_topic_types(self, topics_to_register):
        try:
            if self.__own_state is not None:
                for topicname, topictype, node, nodeuri in topics_to_register:
                    try:
                        if topicname in self.__own_state.topics:
                            own_topictype = self.__own_state.topics[topicname].type
                            if own_topictype not in ['*', None] and topictype not in ['*', None] :
                                if topictype != own_topictype:
                                    if (topicname, node, nodeuri) not in self._topic_type_warnings:
                                        if self.node:
                                            self.node.get_logger().warn("Different topic types detected for topic: %s, own type: %s remote type: %s, local host: %s, remote host: %s" % (topicname, own_topictype, topictype, self.hostname_local, self.name))
                                        self._topic_type_warnings[(topicname, node, nodeuri)] = "local: %s, remote: %s" % (own_topictype, topictype)
                    except Exception as err:
                        import traceback
                        if self.node:
                            self.node.get_logger().warn(str(err))
                            self.node.get_logger().warn(traceback.format_exc())
        except:
            import traceback
            if self.node:
                self.node.get_logger().error("SyncThread[%s] ERROR: %s" % (self.name, traceback.format_exc()))
        finally:
            socket.setdefaulttimeout(None)


    def get_md5warnigs(self):
        with self.__lock_info:
            return dict(self._md5warnings)

    def get_topic_type_warnings(self):
        with self.__lock_info:
            return dict(self._topic_type_warnings)

    def _unreg_on_finish(self):
        with self.__lock_info:
            self.__unregistered = True
            try:
                if self.node:
                    self.node.get_logger().debug("    SyncThread[%s] clear all registrations" % self.name)
                socket.setdefaulttimeout(5)
                own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
                own_master_multi = xmlrpcclient.MultiCall(own_master)
                for topic, _topictype, node, uri in self.__subscriber:
                    if self.node:
                        self.node.get_logger().debug("    SyncThread[%s]   unsibscribe %s [%s]" % (self.name, topic, node))
                    own_master_multi.unregisterSubscriber(node, topic, uri)
                for topic, _topictype, node, uri in self.__publisher:
                    if self.node:
                        self.node.get_logger().debug("    SyncThread[%s]   unadvertise %s [%s]" % (self.name, topic, node))
                    own_master_multi.unregisterPublisher(node, topic, uri)
                for service, serviceuri, node, uri in self.__services:
                    if self.node:
                        self.node.get_logger().debug("    SyncThread[%s]   unregister service %s [%s]" % (self.name, service, node))
                    own_master_multi.unregisterService(node, service, serviceuri)
                if self.node:
                    self.node.get_logger().debug("    SyncThread[%s] execute a MultiCall" % self.name)
                _ = own_master_multi()
                if self.node:
                    self.node.get_logger().debug("    SyncThread[%s] finished" % self.name)
            except:
                if self.node:
                    self.node.get_logger().error("SyncThread[%s] ERROR while ending: %s" % (self.name, traceback.format_exc()))
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