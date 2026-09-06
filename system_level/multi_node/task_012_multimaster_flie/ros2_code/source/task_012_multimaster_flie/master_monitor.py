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

try:
    from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
    from socketserver import ThreadingMixIn
    import io
except ImportError:
    from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
    from SocketServer import ThreadingMixIn
    import cStringIO as io

try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

from datetime import datetime
import getpass
import socket
import subprocess
import sys
import threading
import time
import traceback
import logging

try:
    import xmlrpc.client as xmlrpcclient
except ImportError:
    import xmlrpclib as xmlrpcclient


DISCOVERY_NODE_BASENAME = 'master_discovery'


class MasterConnectionException(Exception):
    '''
    The exception class to handle the connection problems with ROS Master.
    '''
    pass


def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise Exception("remote call failed: %s" % msg)
    return val


class MasterInfo(object):
    """Simplified MasterInfo for ROS2 migration context."""

    def __init__(self, masteruri, mastername=None):
        self.masteruri = masteruri
        self.mastername = mastername or ''
        self.timestamp = time.time()
        self.timestamp_local = time.time()
        self.check_ts = 0
        self._nodes = {}
        self._topics = {}
        self._services = {}
        self._publishers = []  # list of (topic, node, nodeuri)
        self._subscribers = []  # list of (topic, node, nodeuri)
        self._service_list = []  # list of (service, serviceuri, node, nodeuri)
        self._topic_types = {}  # topic -> type

    def getNode(self, name):
        if name not in self._nodes:
            self._nodes[name] = NodeInfo(name)
        return self._nodes[name]

    def getService(self, name):
        if name not in self._services:
            self._services[name] = ServiceInfo(name)
        return self._services[name]

    @property
    def nodes(self):
        return self._nodes

    @property
    def topics(self):
        return self._topics

    @property
    def services(self):
        return self._services

    @property
    def topic_names(self):
        return list(self._topic_types.keys())

    def listedState(self, fi=None):
        t = str(time.time())
        return (t, t, self.masteruri, str(self.mastername), [], [], [], [], [], [])

    def has_local_changes(self, other):
        return True

    def __eq__(self, other):
        if other is None:
            return False
        return self.timestamp == other.timestamp

    def __ne__(self, other):
        return not self.__eq__(other)


class NodeInfo(object):
    def __init__(self, name):
        self.name = name
        self.uri = None
        self.pid = None
        self.masteruri = None


class ServiceInfo(object):
    def __init__(self, name):
        self.name = name
        self.uri = None
        self.type = None
        self.masteruri = None


class TopicInfo(object):
    def __init__(self, name, topic_type=None):
        self.name = name
        self.type = topic_type
        self.publishers = []
        self.subscribers = []


class RPCThreading(ThreadingMixIn, SimpleXMLRPCServer):
    daemon_threads = True

    def __init__(self, addr, requestHandler=SimpleXMLRPCRequestHandler,
                 logRequests=True, allow_none=False, encoding=None, bind_and_activate=True):
        SimpleXMLRPCServer.__init__(self, addr, requestHandler=requestHandler,
                 logRequests=logRequests, allow_none=allow_none, encoding=encoding, bind_and_activate=bind_and_activate)


class RPCThreadingV6(ThreadingMixIn, SimpleXMLRPCServer):
    address_family = socket.AF_INET6
    daemon_threads = True

    def __init__(self, addr, requestHandler=SimpleXMLRPCRequestHandler,
                 logRequests=True, allow_none=False, encoding=None, bind_and_activate=True):
        SimpleXMLRPCServer.__init__(self, addr, requestHandler=requestHandler,
                 logRequests=logRequests, allow_none=allow_none, encoding=encoding, bind_and_activate=bind_and_activate)


class MasterMonitor(object):
    '''
    This class provides methods to get the state from the ROS master using his
    RPC API and test for changes.
    '''

    MAX_PING_SEC = 10.0
    INTERVAL_UPDATE_LAUNCH_URIS = 15.0

    def __init__(self, masteruri, ros_node_name='/master_monitor', mastername=None,
                 rpcport=11611, do_retry=True, ipv6=False, rpc_addr=''):
        self._state_access_lock = threading.RLock()
        self._create_access_lock = threading.RLock()
        self._lock = threading.RLock()
        self.__masteruri = masteruri
        self.__new_master_state = None
        self.__masteruri_rpc = None
        self.__mastername = mastername
        self.__cached_nodes = dict()
        self.__cached_services = dict()
        self.ros_node_name = ros_node_name

        if self.__mastername is None:
            self.__mastername = self.getMastername()

        self.__master_state = None
        self.rpcport = rpcport

        self._printed_errors = dict()
        self._last_clearup_ts = time.time()
        self._master_errors = list()

        self._master = xmlrpcclient.ServerProxy(self.__masteruri)

        self.logger = logging.getLogger('MasterMonitor')

    def _succeed(self, args):
        """Validate XML-RPC response from the Master."""
        return _succeed(args)

    def shutdown(self):
        pass

    def is_running(self):
        return True

    def getListedMasterInfo(self):
        t = str(time.time())
        result = (t, t, self.getMasteruri(), str(self.getMastername()), [], [], [], [], [], [])
        if not (self.__master_state is None):
            try:
                with self._state_access_lock:
                    result = self.__master_state.listedState()
            except:
                print(traceback.format_exc())
        return result

    def getListedMasterInfoFiltered(self, filter_list):
        t = str(time.time())
        result = (t, t, self.getMasteruri(), str(self.getMastername()), [], [], [], [], [], [])
        if not (self.__master_state is None):
            try:
                with self._state_access_lock:
                    result = self.__master_state.listedState()
            except:
                print(traceback.format_exc())
        return result

    def getCurrentState(self):
        with self._state_access_lock:
            return self.__master_state

    def updateState(self, clear_cache=False):
        '''
        Gets the state from the ROS master using XML-RPC calls and updates
        the internal MasterInfo representation.

        :return: The new MasterInfo state object

        :rtype: MasterInfo or None
        '''
        try:
            self.__new_master_state = MasterInfo(self.getMasteruri(), self.getMastername())

            # Clear caches if requested
            if clear_cache:
                with self._lock:
                    self.__cached_nodes.clear()
                    self.__cached_services.clear()

            # Create a proxy to the ROS Master
            master = xmlrpcclient.ServerProxy(self.getMasteruri())

            # Retrieve topic types from the Master using getTopicTypes
            topic_types = self._succeed(master.getTopicTypes(self.ros_node_name))

            # Build a dictionary mapping topic name -> topic type
            topic_type_dict = {}
            for topic_name, topic_type in topic_types:
                topic_type_dict[topic_name] = topic_type

            # Store ALL topic types from getTopicTypes into the state
            for topic_name, topic_type in topic_types:
                self.__new_master_state._topic_types[topic_name] = topic_type
                if topic_name not in self.__new_master_state._topics:
                    self.__new_master_state._topics[topic_name] = TopicInfo(topic_name, topic_type)

            # Retrieve the full system state from the Master using getSystemState
            system_state = self._succeed(master.getSystemState(self.ros_node_name))

            # system_state is [publishers, subscribers, service_providers]
            publishers = system_state[0]
            subscribers = system_state[1]
            services = system_state[2]

            # Process publishers: iterate through the system state
            for topic, nodes in publishers:
                topic_type = topic_type_dict.get(topic, '')
                # Store topic type mapping
                self.__new_master_state._topic_types[topic] = topic_type
                # Create or update topic info
                if topic not in self.__new_master_state._topics:
                    self.__new_master_state._topics[topic] = TopicInfo(topic, topic_type)
                else:
                    self.__new_master_state._topics[topic].type = topic_type
                for node in nodes:
                    self.__new_master_state._topics[topic].publishers.append(node)
                    # Ensure node exists in state
                    self.__new_master_state.getNode(node)
                    self.__new_master_state._publishers.append((topic, node, None))

            # Process subscribers
            for topic, nodes in subscribers:
                topic_type = topic_type_dict.get(topic, '')
                if topic not in self.__new_master_state._topics:
                    self.__new_master_state._topics[topic] = TopicInfo(topic, topic_type)
                for node in nodes:
                    self.__new_master_state._topics[topic].subscribers.append(node)
                    self.__new_master_state.getNode(node)
                    self.__new_master_state._subscribers.append((topic, node, None))

            # Process services
            for service, nodes in services:
                for node in nodes:
                    self.__new_master_state.getNode(node)
                    svc_info = self.__new_master_state.getService(service)
                    self.__new_master_state._service_list.append((service, None, node, None))

            # Update timestamp
            self.__new_master_state.timestamp = time.time()

            with self._create_access_lock:
                return self.__new_master_state

        except Exception as e:
            self.logger.warning("MasterMonitor.updateState() error: %s" % str(e))
            if self.__new_master_state is None:
                self.__new_master_state = MasterInfo(self.getMasteruri(), self.getMastername())
            return self.__new_master_state

    def _limited_log(self, provider, msg, level=logging.WARN):
        if provider not in self._printed_errors:
            self._printed_errors[provider] = dict()
        if msg not in self._printed_errors[provider]:
            self._printed_errors[provider][msg] = time.time()
            self.logger.log(level, "MasterMonitor[%s]: %s" % (provider, msg))

    def _clearup_cached_logs(self, age=300):
        cts = time.time()
        with self._lock:
            for p, msgs in list(self._printed_errors.items()):
                for msg, ts in list(msgs.items()):
                    if cts - ts > age:
                        del self._printed_errors[p][msg]
                if not self._printed_errors[p]:
                    del self._printed_errors[p]

    def getMasteruri(self):
        if self.__masteruri_rpc is None:
            try:
                master = xmlrpcclient.ServerProxy(self.__masteruri)
                code, message, self.__masteruri_rpc = master.getUri(self.ros_node_name)
            except:
                return self.__masteruri
        return self.__masteruri_rpc if self.__masteruri_rpc is not None else self.__masteruri

    def getMastername(self):
        if self.__mastername is None:
            try:
                uri = self.getMasteruri()
                parsed = urlparse(uri)
                self.__mastername = parsed.hostname
                if parsed.port and parsed.port != 11311:
                    self.__mastername = '%s_%d' % (self.__mastername, parsed.port)
            except:
                self.__mastername = 'unknown'
        return self.__mastername

    def getMasterContacts(self):
        t = 0
        if self.__master_state is not None:
            with self._state_access_lock:
                t = self.__master_state.timestamp
        return ('%.9f' % t, str(self.getMasteruri()), str(self.getMastername()), self.ros_node_name, '')

    def getMasterErrors(self):
        return (str(self.getMasteruri()), self._master_errors)

    def getCurrentTime(self):
        return (str(self.getMasteruri()), time.time())

    def setTime(self, timestamp):
        dtime = datetime.fromtimestamp(timestamp)
        args = ['sudo', '-n', '/bin/date', '-s', '%s' % dtime]
        subp = subprocess.Popen(args, stderr=subprocess.PIPE)
        success = True
        result_err = ''
        if subp.stderr is not None:
            result_err = subp.stderr.read()
            if result_err:
                success = False
        return (str(self.getMasteruri()), success, time.time(), result_err)

    def getUser(self):
        return (str(self.getMasteruri()), getpass.getuser())

    def checkState(self, clear_cache=False):
        result = False
        s = self.updateState(clear_cache)
        with self._create_access_lock:
            do_update = False
            with self._state_access_lock:
                if s != self.__master_state:
                    do_update = True
                if self.__master_state is not None and s.timestamp < self.__master_state.timestamp:
                    do_update = True
                    result = True
            if do_update:
                with self._state_access_lock:
                    ts_local = self.__new_master_state.timestamp_local
                    if self.__master_state is not None and not self.__master_state.has_local_changes(s):
                        ts_local = self.__master_state.timestamp_local
                    self.__master_state = self.__new_master_state
                    self.__master_state.timestamp_local = ts_local
                    result = True
            if self.__master_state is not None:
                self.__master_state.check_ts = self.__new_master_state.timestamp
            return result

    def reset(self):
        with self._state_access_lock:
            if self.__master_state is not None:
                del self.__master_state
            self.__master_state = None

    def update_master_errors(self, error_list):
        self._master_errors = list(error_list)