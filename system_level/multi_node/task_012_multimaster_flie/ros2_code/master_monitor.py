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
    import cStringIO as io  # python 2 compatibility
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse  # python 2 compatibility
from datetime import datetime
import getpass
import rclpy
from rclpy.node import Node
import socket
import subprocess
import sys
import threading
import time
import traceback
try:
    import xmlrpc.client as xmlrpcclient
except ImportError:
    import xmlrpclib as xmlrpcclient  # python 2 compatibility

from . import interface_finder

from .common import masteruri_from_ros, get_hostname
from .common import gen_pattern
from .filter_interface import FilterInterface
from .master_info import MasterInfo


try:  # to avoid the problems with autodoc on ros.org/wiki site
    from fkie_multimaster_msgs.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster, SyncMasterInfo, SyncTopicInfo, SyncServiceInfo
    from fkie_multimaster_msgs.srv import DiscoverMasters, GetSyncInfo
except:
    pass


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
    MAX_PING_SEC = 10.0
    INTERVAL_UPDATE_LAUNCH_URIS = 15.0

    def __init__(self, rpcport=11611, do_retry=True, ipv6=False, rpc_addr='', node=None):
        self.node = node
        self._state_access_lock = threading.RLock()
        self._create_access_lock = threading.RLock()
        self._lock = threading.RLock()
        self.__masteruri = masteruri_from_ros()
        self.__new_master_state = None
        self.__masteruri_rpc = None
        self.__mastername = None
        self.__cached_nodes = dict()
        self.__cached_services = dict()
        self.ros_node_name = str(self.node.get_name()) if self.node else 'master_monitor'
        
        self.__mastername = self.getMastername()

        self.__master_state = None
        self.rpcport = rpcport

        self._printed_errors = dict()
        self._last_clearup_ts = time.time()

        self._master_errors = list()
        self.ready = False
        while not self.ready and rclpy.ok():
            try:
                RPCClass = RPCThreading
                if ipv6:
                    RPCClass = RPCThreadingV6
                self.rpcServer = RPCClass((rpc_addr, rpcport), logRequests=False, allow_none=True)
                if self.node:
                    self.node.get_logger().info("Start RPC-XML Server at %s" % str(self.rpcServer.server_address))
                self.rpcServer.register_introspection_functions()
                self.rpcServer.register_function(self.getListedMasterInfo, 'masterInfo')
                self.rpcServer.register_function(self.getListedMasterInfoFiltered, 'masterInfoFiltered')
                self.rpcServer.register_function(self.getMasterContacts, 'masterContacts')
                self.rpcServer.register_function(self.getMasterErrors, 'masterErrors')
                self.rpcServer.register_function(self.getCurrentTime, 'getCurrentTime')
                self.rpcServer.register_function(self.setTime, 'setTime')
                self.rpcServer.register_function(self.getTopicsMd5sum, 'getTopicsMd5sum')
                self.rpcServer.register_function(self.getUser, 'getUser')
                self._rpcThread = threading.Thread(target=self.rpcServer.serve_forever)
                self._rpcThread.setDaemon(True)
                self._rpcThread.start()
                self.ready = True
            except socket.error as e:
                if not do_retry:
                    raise Exception("Error while start RPC-XML server on port %d: %s\nIs a Node Manager already running?" % (rpcport, e))
                if self.node:
                    self.node.get_logger().warn("Error while start RPC-XML server on port %d: %s\nTry again..." % (rpcport, e))
                time.sleep(1)
            except:
                print(traceback.format_exc())
                if not do_retry:
                    raise

        self._master = xmlrpcclient.ServerProxy(self.getMasteruri())
        self._re_hide_nodes = gen_pattern([], 'hide_nodes')
        self._re_hide_topics = gen_pattern([], 'hide_topics')
        self._re_hide_services = gen_pattern([], 'hide_services')
        
        self._update_launch_uris_lock = threading.RLock()
        self.__launch_uris = {}
        self._timer_update_launch_uris = None
        self._update_launch_uris()

    def __update_param(self, key, value):
        with self._update_launch_uris_lock:
            try:
                if value:
                    self.__launch_uris[key] = value
                else:
                    del self.__launch_uris[key]
            except:
                pass

    def shutdown(self):
        if self._timer_update_launch_uris is not None:
            try:
                self._timer_update_launch_uris.cancel()
            except Exception:
                pass
        if hasattr(self, 'rpcServer'):
            if self.node:
                self.node.get_logger().info("shutdown own RPC server")
            self.rpcServer.shutdown()
            del self.rpcServer.socket
            del self.rpcServer

    def is_running(self):
        return hasattr(self, 'rpcServer')

    def _update_launch_uris(self, params={}):
        with self._update_launch_uris_lock:
            if params:
                self.__launch_uris = params
            try:
                socket.setdefaulttimeout(3.0)
                for key, value in self.__launch_uris.items():
                    try:
                        launch_server = xmlrpcclient.ServerProxy(value)
                        c, m, pid = launch_server.get_pid()
                    except:
                        try:
                            master = xmlrpcclient.ServerProxy(self.getMasteruri())
                            master.deleteParam(self.ros_node_name, key)
                        except:
                            pass
            finally:
                socket.setdefaulttimeout(None)
                if rclpy.ok():
                    self._timer_update_launch_uris = threading.Timer(self.INTERVAL_UPDATE_LAUNCH_URIS, self._update_launch_uris)
                    self._timer_update_launch_uris.start()

    def _getNodePid(self, nodes):
        for (nodename, uri) in nodes.items():
            if uri is not None:
                pid = None
                try:
                    with self._lock:
                        if nodename in self.__cached_nodes:
                            if time.time() - self.__cached_nodes[nodename][2] < self.MAX_PING_SEC:
                                return
                    socket.setdefaulttimeout(0.7)
                    node = xmlrpcclient.ServerProxy(uri)
                    pid = _succeed(node.getPid(self.ros_node_name))
                except (Exception, socket.error) as e:
                    with self._lock:
                        self._limited_log(nodename, "can't get PID: %s" % str(e), level=1)
                    master = xmlrpcclient.ServerProxy(self.getMasteruri())
                    code, message, new_uri = master.lookupNode(self.ros_node_name, nodename)
                    with self._lock:
                        self.__new_master_state.getNode(nodename).uri = None if (code == -1) else new_uri
                        if code == -1:
                            self._limited_log(nodename, "can't update contact information. ROS master responds with: %s" % message)
                        try:
                            del self.__cached_nodes[nodename]
                        except:
                            pass
                else:
                    with self._lock:
                        self.__new_master_state.getNode(nodename).pid = pid
                        self.__cached_nodes[nodename] = (uri, pid, time.time())
                finally:
                    socket.setdefaulttimeout(None)

    def _getServiceInfo(self, services):
        for (service, uri) in services.items():
            with self._lock:
                if service in self.__cached_services:
                    if time.time() - self.__cached_services[service][2] < self.MAX_PING_SEC:
                        return
            if uri is not None:
                dest_addr = dest_port = None
                try:
                    parsed = urlparse(uri)
                    dest_addr = parsed.hostname
                    dest_port = parsed.port
                except:
                    continue
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    s.settimeout(0.5)
                    s.connect((dest_addr, dest_port))
                    with self._lock:
                        self.__new_master_state.getService(service).type = 'unknown'
                        self.__cached_services[service] = (uri, 'unknown', time.time())
                except socket.error:
                    with self._lock:
                        try:
                            del self.__cached_services[service]
                        except:
                            pass
                except:
                    with self._lock:
                        self._limited_log(service, "can't get service type: %s" % traceback.format_exc(), level=1)
                    with self._lock:
                        try:
                            del self.__cached_services[service]
                        except:
                            pass
                    pass
                finally:
                    if s is not None:
                        s.close()

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
                    fi = FilterInterface.from_list(filter_list)
                    fi.set_hide_pattern(self._re_hide_nodes, self._re_hide_topics, self._re_hide_services)
                    result = self.__master_state.listedState(fi)
            except:
                print(traceback.format_exc())
        return result

    def getCurrentState(self):
        with self._state_access_lock:
            return self.__master_state

    def updateState(self, clear_cache=False):
        try:
            master = xmlrpcclient.ServerProxy(self.getMasteruri())
            
            state_response = master.getSystemState(self.ros_node_name)
            publishers, subscribers, services = _succeed(state_response)
            
            topic_types_response = master.getTopicTypes(self.ros_node_name)
            topic_types = _succeed(topic_types_response)
            topic_type_dict = {topic: ttype for topic, ttype in topic_types}
            
            with self._create_access_lock:
                self.__new_master_state = MasterInfo(self.getMasteruri(), self.getMastername())
                
                for topic, nodes in publishers:
                    ttype = topic_type_dict.get(topic, 'None')
                    for node in nodes:
                        self.__new_master_state.add_publisher(node, topic, ttype)
                        
                for topic, nodes in subscribers:
                    ttype = topic_type_dict.get(topic, 'None')
                    for node in nodes:
                        self.__new_master_state.add_subscriber(node, topic, ttype)
                        
                for service, nodes in services:
                    for node in nodes:
                        self.__new_master_state.add_service(node, service)
                        
            return True
        except Exception as e:
            self._limited_log("updateState", "Communication failure: %s" % str(e), level=2)
            return False

    def _limited_log(self, provider, msg, level=2):
        if provider not in self._printed_errors:
            self._printed_errors[provider] = dict()
        if msg not in self._printed_errors[provider]:
            self._printed_errors[provider][msg] = time.time()
            if self.node:
                if level == 1:
                    self.node.get_logger().debug("MasterMonitor[%s]: %s" % (provider, msg))
                elif level == 2:
                    self.node.get_logger().warn("MasterMonitor[%s]: %s" % (provider, msg))
                elif level == 3:
                    self.node.get_logger().error("MasterMonitor[%s]: %s" % (provider, msg))
                elif level == 4:
                    self.node.get_logger().fatal("MasterMonitor[%s]: %s" % (provider, msg))
                else:
                    self.node.get_logger().info("MasterMonitor[%s]: %s" % (provider, msg))

    def _clearup_cached_logs(self, age=300):
        cts = time.time()
        with self._lock:
            for p, msgs in list(self._printed_errors.items()):
                for msg, ts in list(msgs.items()):
                    if cts - ts > age:
                        del self._printed_errors[p][msg]
                if not self._printed_errors[p]:
                    del self._printed_errors[p]

    def updateSyncInfo(self):
        def getNodeuri(nodename, publisher, subscriber, services):
            for p in publisher:
                if nodename == p.node:
                    return p.nodeuri
            for p in subscriber:
                if nodename == p.node:
                    return p.nodeuri
            for s in services:
                if nodename == s.node:
                    return s.nodeuri
            return None

        with self._create_access_lock:
            master_state = self.__new_master_state
            sync_info = None
            for name, service in master_state.services.items():
                if service.name.endswith('get_sync_info'):
                    if get_hostname(self.getMasteruri()) == get_hostname(service.uri):
                        socket.setdefaulttimeout(3)
                        try:
                            pass
                        except Exception as e:
                            if self.node:
                                self.node.get_logger().warn("ERROR Service call 'get_sync_info' failed: %s" % str(e))
                        finally:
                            socket.setdefaulttimeout(None)

            if sync_info:
                for m in sync_info.hosts:
                    for n in m.nodes:
                        try:
                            nuri = getNodeuri(n, m.publisher, m.subscriber, m.services)
                            state_node = master_state.getNode(n)
                            if state_node is not None and (state_node.uri == nuri or nuri is None):
                                state_node.masteruri = m.masteruri
                        except:
                            pass
                    for s in m.services:
                        try:
                            state_service = master_state.getService(s.service)
                            if state_service is not None and state_service.uri == s.serviceuri:
                                state_service.masteruri = m.masteruri
                        except:
                            pass

    def getMasteruri(self):
        code = -1
        if self.__masteruri_rpc is None:
            master = xmlrpcclient.ServerProxy(self.__masteruri)
            try:
                code, message, self.__masteruri_rpc = master.getUri(self.ros_node_name)
            except:
                pass
        return self.__masteruri_rpc if code >= 0 or self.__masteruri_rpc is not None else self.__masteruri

    def getMastername(self):
        if self.__mastername is None:
            try:
                self.__mastername = get_hostname(self.getMasteruri())
                try:
                    master_port = urlparse(self.__masteruri).port
                    if master_port != 11311:
                        self.__mastername = '%s_%d' % (self.__mastername, master_port)
                except:
                    pass
            except:
                pass
        return self.__mastername

    def getMasterContacts(self):
        t = 0
        if self.__master_state is not None:
            with self._state_access_lock:
                t = self.__master_state.timestamp
        return ('%.9f' % t, str(self.getMasteruri()), str(self.getMastername()), self.ros_node_name, "http://localhost:%d" % self.rpcport)

    def getMasterErrors(self):
        return (str(self.getMasteruri()), self._master_errors)

    def getCurrentTime(self):
        return (str(self.getMasteruri()), time.time())

    def setTime(self, timestamp):
        dtime = datetime.fromtimestamp(timestamp)
        args = ['sudo', '-n', '/bin/date', '-s', '%s' % dtime]
        if self.node:
            self.node.get_logger().info('Set time: %s' % args)
        subp = subprocess.Popen(args, stderr=subprocess.PIPE)
        success = True
        result_err = ''
        if subp.stderr is not None:
            result_err = subp.stderr.read()
            if result_err:
                success = False
        return (str(self.getMasteruri()), success, time.time(), result_err)

    def getTopicsMd5sum(self, topic_types):
        topic_list = []
        for ttype in topic_types:
            try:
                entry = (ttype, "unknown")
                topic_list.append(entry)
            except Exception as err:
                if self.node:
                    self.node.get_logger().warn(str(err))
        return topic_list

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
                    timejump_msg = "Timejump into past detected! Restart all ROS nodes, includes master_discovery, please!"
                    if self.node:
                        self.node.get_logger().warn(timejump_msg)
                    if timejump_msg not in self._master_errors:
                        self._master_errors.append(timejump_msg)
                    self._exit_timer = threading.Timer(5.0, self._timejump_exit)
                    self._exit_timer.start()
            if do_update:
                self.updateSyncInfo()
                with self._state_access_lock:
                    ts_local = self.__new_master_state.timestamp_local
                    if self.__master_state is not None and not self.__master_state.has_local_changes(s):
                        ts_local = self.__master_state.timestamp_local
                    self.__master_state = self.__new_master_state
                    self.__master_state.timestamp_local = ts_local
                    result = True
            self.__master_state.check_ts = self.__new_master_state.timestamp
            return result

    def _timejump_exit(self):
        if self.node:
            self.node.get_logger().warn('Shutdown yourself to avoid system instability because of time jump into past!\n')
        sys.exit(1)

    def reset(self):
        with self._state_access_lock:
            if self.__master_state is not None:
                del self.__master_state
            self.__master_state = None

    def update_master_errors(self, error_list):
        self._master_errors = list(error_list)