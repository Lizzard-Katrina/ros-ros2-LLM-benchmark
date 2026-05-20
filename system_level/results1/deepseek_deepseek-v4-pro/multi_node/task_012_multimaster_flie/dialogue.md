# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: master_monitor.py
----------------------------
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
    from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
    from SocketServer import ThreadingMixIn
    import cStringIO as io  # python 2 compatibility
except ImportError:
    from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
    from socketserver import ThreadingMixIn
    import io
try:
    from urlparse import urlparse  # python 2 compatibility
except ImportError:
    from urllib.parse import urlparse
from datetime import datetime
import getpass
import roslib.network
import roslib.message
import rospy
import socket
import subprocess
import sys
import threading
import time
import traceback
try:
    import xmlrpclib as xmlrpcclient  # python 2 compatibility
except ImportError:
    import xmlrpc.client as xmlrpcclient

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
    # When inheriting from ThreadingMixIn for threaded connection behavior, you should explicitly
    # declare how you want your threads to behave on an abrupt shutdown. The ThreadingMixIn class
    # defines an attribute daemon_threads, which indicates whether or not the server should wait
    # for thread termination. You should set the flag explicitly if you would like threads to
    # behave autonomously; the default is False, meaning that Python will not exit until all
    # threads created by ThreadingMixIn have exited.
    daemon_threads = True

    def __init__(self, addr, requestHandler=SimpleXMLRPCRequestHandler,
                 logRequests=True, allow_none=False, encoding=None, bind_and_activate=True):
        SimpleXMLRPCServer.__init__(self, addr, requestHandler=requestHandler,
                 logRequests=logRequests, allow_none=allow_none, encoding=encoding, bind_and_activate=bind_and_activate)


class RPCThreadingV6(ThreadingMixIn, SimpleXMLRPCServer):
    address_family = socket.AF_INET6
    # When inheriting from ThreadingMixIn for threaded connection behavior, you should explicitly
    # declare how you want your threads to behave on an abrupt shutdown. The ThreadingMixIn class
    # defines an attribute daemon_threads, which indicates whether or not the server should wait
    # for thread termination. You should set the flag explicitly if you would like threads to
    # behave autonomously; the default is False, meaning that Python will not exit until all
    # threads created by ThreadingMixIn have exited.
    daemon_threads = True

    def __init__(self, addr, requestHandler=SimpleXMLRPCRequestHandler,
                 logRequests=True, allow_none=False, encoding=None, bind_and_activate=True):
        SimpleXMLRPCServer.__init__(self, addr, requestHandler=requestHandler,
                 logRequests=logRequests, allow_none=allow_none, encoding=encoding, bind_and_activate=bind_and_activate)


class MasterMonitor(object):
    '''
    This class provides methods to get the state from the ROS master using his
    RPC API and test for changes. Furthermore an XML-RPC server will be created
    to offer the complete current state of the ROS master by one method call.

    :param rpcport: the port number for the XML-RPC server

    :type rpcport:  int

    :param do_retry: retry to create XML-RPC server

    :type do_retry: bool

    :see: :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getCurrentState()`, respectively
          :mod:`fkie_master_discovery.master_monitor.MasterMonitor.updateState()`

    :RPC Methods:
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getListedMasterInfo()` or
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getMasterContacts()` as RPC:
        ``masterInfo()`` and ``masterContacts()``
    '''

    MAX_PING_SEC = 10.0
    ''' The time to update the node URI, ID or service URI (Default: ``10.0``)'''

    INTERVAL_UPDATE_LAUNCH_URIS = 15.0

    def __init__(self, rpcport=11611, do_retry=True, ipv6=False, rpc_addr=''):
        '''
        Initialize method. Creates an XML-RPC server on given port and starts this
        in its own thread.

        :param rpcport: the port number for the XML-RPC server

        :type rpcport:  int

        :param do_retry: retry to create XML-RPC server

        :type do_retry: bool

        :param ipv6: Use ipv6

        :type ipv6: bool
        '''
        self._state_access_lock = threading.RLock()
        self._create_access_lock = threading.RLock()
        self._lock = threading.RLock()
        self.__masteruri = masteruri_from_ros()
        self.__new_master_state = None
        self.__masteruri_rpc = None
        self.__mastername = None
        self.__cached_nodes = dict()
        self.__cached_services = dict()
        self.ros_node_name = str(rospy.get_name())
        if rospy.has_param('~name'):
            self.__mastername = rospy.get_param('~name')
        self.__mastername = self.getMastername()
        rospy.set_param('/mastername', self.__mastername)

        self.__master_state = None
        '''the current state of the ROS master'''
        self.rpcport = rpcport
        '''the port number of the RPC server'''

        self._printed_errors = dict()
        self._last_clearup_ts = time.time()

        self._master_errors = list()
        # Create an XML-RPC server
        self.ready = False
        while not self.ready and not rospy.is_shutdown():
            try:
                RPCClass = RPCThreading
                if ipv6:
                    RPCClass = RPCThreadingV6
                self.rpcServer = RPCClass((rpc_addr, rpcport), logRequests=False, allow_none=True)
                rospy.loginfo("Start RPC-XML Server at %s", self.rpcServer.server_address)
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
                rospy.logwarn("Error while start RPC-XML server on port %d: %s\nTry again..." % (rpcport, e))
                time.sleep(1)
            except:
                print(traceback.format_exc())
                if not do_retry:
                    raise

        self._master = xmlrpcclient.ServerProxy(self.getMasteruri())
        # Hide parameter
        self._re_hide_nodes = gen_pattern(rospy.get_param('~hide_nodes', []), 'hide_nodes')
        self._re_hide_topics = gen_pattern(rospy.get_param('~hide_topics', []), 'hide_topics')
        self._re_hide_services = gen_pattern(rospy.get_param('~hide_services', []), 'hide_services')
        # === UPDATE THE LAUNCH URIS Section ===
        # subscribe to get parameter updates
        rospy.loginfo("Subscribe to parameter `/roslaunch/uris`")
        self.__mycache_param_server = rospy.impl.paramserver.get_param_server_cache()
        # HACK: use own method to get the updates also for parameters in the subgroup
        self.__mycache_param_server.update = self.__update_param
        # first access, make call to parameter server
        self._update_launch_uris_lock = threading.RLock()
        self.__launch_uris = {}
        code, msg, value = self._master.subscribeParam(self.ros_node_name, rospy.get_node_uri(), '/roslaunch/uris')
        # the new timer will be created in self._update_launch_uris()
        self._timer_update_launch_uris = None
        if code == 1:
            for k, v in value.items():
                self.__launch_uris[roslib.names.ns_join('/roslaunch/uris', k)] = v
        self._update_launch_uris()
        # === END: UPDATE THE LAUNCH URIS Section ===

    def __update_param(self, key, value):
        # updates the /roslaunch/uris parameter list
        with self._update_launch_uris_lock:
            try:
                if value:
                    self.__launch_uris[key] = value
                else:
                    del self.__launch_uris[key]
            except:
                pass

    def shutdown(self):
        '''
        Shutdown the RPC Server.
        '''
        if self._timer_update_launch_uris is not None:
            try:
                self._timer_update_launch_uris.cancel()
            except Exception:
                pass
        if hasattr(self, 'rpcServer'):
            if self._master is not None:
                rospy.loginfo("Unsubscribe from parameter `/roslaunch/uris`")
                try:
                    self._master.unsubscribeParam(self.ros_node_name, rospy.get_node_uri(), '/roslaunch/uris')
                except Exception as e:
                    rospy.logwarn("Error while unsubscribe from `/roslaunch/uris`: %s" % e)
            rospy.loginfo("shutdown own RPC server")
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
                        # contact the launch server
                        launch_server = xmlrpcclient.ServerProxy(value)
                        c, m, pid = launch_server.get_pid()
                    except:
                        try:
                            # remove the parameter from parameter server on error
                            master = xmlrpcclient.ServerProxy(self.getMasteruri())
                            master.deleteParam(self.ros_node_name, key)
                        except:
                            pass
            finally:
                socket.setdefaulttimeout(None)
                # create the new timer
                if not rospy.is_shutdown():
                    self._timer_update_launch_uris = threading.Timer(self.INTERVAL_UPDATE_LAUNCH_URIS, self._update_launch_uris)
                    self._timer_update_launch_uris.start()

    def _getNodePid(self, nodes):
        '''
        Gets process id of the node.
        This method blocks until the info is retrieved or socket timeout is reached (0.7 seconds).

        :param nodename: the name of the node

        :type nodename: str

        :param uri: the uri of the node

        :type uri: str
        '''
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
                        self._limited_log(nodename, "can't get PID: %s" % str(e), level=rospy.DEBUG)
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
#          print "_getNodePid _lock RET", threading.current_thread()
                finally:
                    socket.setdefaulttimeout(None)

    def _getServiceInfo(self, services):
        '''
        Gets service info through the RPC interface of the service.
        This method blocks until the info is retrieved or socket timeout is reached (0.5 seconds).

        :param service: the name of the service

        :type service: str

        :param uri: the uri of the service

        :type uri: str
        '''
        for (service, uri) in services.items():
            with self._lock:
                if service in self.__cached_services:
                    if time.time() - self.__cached_services[service][2] < self.MAX_PING_SEC:
                        return
            if uri is not None:
                dest_addr = dest_port = None
                try:
                    dest_addr, dest_port = rospy.parse_rosrpc_uri(uri)
                except:
                    continue
        #      raise ROSServiceException("service [%s] has an invalid RPC URI [%s]"%(service, uri))
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    # connect to service and probe it to get the headers
                    s.settimeout(0.5)
                    s.connect((dest_addr, dest_port))
                    header = {'probe': '1', 'md5sum': '*',
                              'callerid': self.ros_node_name, 'service': service}
                    roslib.network.write_ros_handshake_header(s, header)
                    buf = io.StringIO() if sys.version_info < (3, 0) else io.BytesIO()
                    stype = roslib.network.read_ros_handshake_header(s, buf, 2048)
                    with self._lock:
                        self.__new_master_state.getService(service).type = stype['type']
                        self.__cached_services[service] = (uri, stype['type'], time.time())
                except socket.error:
                    with self._lock:
                        try:
                            del self.__cached_services[service]
                        except:
                            pass
        #      raise ROSServiceIOException("Unable to communicate with service [%s], address [%s]"%(service, uri))
                except:
                    with self._lock:
                        self._limited_log(service, "can't get service type: %s" % traceback.format_exc(), level=rospy.DEBUG)
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
        '''
        :return: a extended ROS Master State.

        :rtype:  :mod:`fkie_master_discovery.master_info.MasterInfo.listedState()` for result type
        '''
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
        '''
        :return: a extended filtered ROS Master State.

        :rtype:  :mod:`fkie_master_discovery.master_info.MasterInfo.listedState()` for result type
        '''
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
        '''
        :return: The current ROS Master State

        :rtype: :mod:`fkie_master_discovery.master_info.MasterInfo` or ``None``
        '''
        with self._state_access_lock:
            return self.__master_state

    def updateState(self, clear_cache=False):
        # TODO [Task_012_MasterMonitor]: 
        # Goal: Synchronize the local 'MasterInfo' object with the actual state of the ROS Master.
        #
        # Requirements:
        # 1. Establish communication with the ROS Master at the given 'masteruri' to retrieve 
        #    the current system state (all publishers, subscribers, and services).
        # 2. Extract and map the message types for all active topics.
        # 3. Update the internal state representation ('self.__new_master_state') such that it 
        #    accurately reflects which nodes are associated with which topics and services.
        # 4. Implement robust error handling for communication failures, ensuring the function 
        #    returns a boolean status indicating success or failure.
        #
        # STYLE CONSTRAINTS (CRITICAL for System Integration):
        # - You MUST use 'self._succeed()' to validate every XML-RPC response from the Master.
        # - When iterating through the system state, use the exact loop structure: 
        #   'for topic, nodes in publishers:' to ensure compatibility with state analysis tools.
        # - Do not use hardcoded strings for node identification.
        # END OF TODO
    def _limited_log(self, provider, msg, level=rospy.WARN):
        if provider not in self._printed_errors:
            self._printed_errors[provider] = dict()
        if msg not in self._printed_errors[provider]:
            self._printed_errors[provider][msg] = time.time()
            if level == rospy.DEBUG:
                rospy.logdebug("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rospy.INFO:
                rospy.loginfo("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rospy.WARN:
                rospy.logwarn("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rospy.ERROR:
                rospy.logerr("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rospy.FATAL:
                rospy.logfatal("MasterMonitor[%s]: %s" % (provider, msg))

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
        '''
        This method can be called to update the origin ROS master URI of the nodes
        and services in new ``master_state``. This is only need, if a synchronization is
        running. The synchronization service will be detect automatically by searching
        for the service ending with ``get_sync_info``. The method will be called by
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.checkState()`.
        '''
        # 'print "updateSyncInfo _create_access_lock try...", threading.current_thread()

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
            # get synchronization info, if sync node is running
            # to determine the origin ROS MASTER URI of the nodes
            for name, service in master_state.services.items():
                if service.name.endswith('get_sync_info'):
                    if get_hostname(self.getMasteruri()) == get_hostname(service.uri):
                        socket.setdefaulttimeout(3)
                        get_sync_info = rospy.ServiceProxy(service.name, GetSyncInfo)
                        try:
                            sync_info = get_sync_info()
                        except rospy.ServiceException as e:
                            rospy.logwarn("ERROR Service call 'get_sync_info' failed: %s", str(e))
                        finally:
                            socket.setdefaulttimeout(None)

            # update the origin ROS MASTER URI of the nodes, if sync node is running
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
        '''
        Requests the ROS master URI from the ROS master through the RPC interface and
        returns it.

        :return: ROS master URI

        :rtype: str or ``None``
        '''
        code = -1
        if self.__masteruri_rpc is None:
            master = xmlrpcclient.ServerProxy(self.__masteruri)
            code, message, self.__masteruri_rpc = master.getUri(self.ros_node_name)
        return self.__masteruri_rpc if code >= 0 or self.__masteruri_rpc is not None else self.__masteruri

    def getMastername(self):
        '''
        Returns the name of the master. If no name is set, the hostname of the
        ROS master URI will be extracted.

        :return: the name of the ROS master

        :rtype: str or ``None``
        '''
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
        '''
        The RPC method called by XML-RPC server to request the master contact information.

        :return: (``timestamp of the ROS master state``, ``ROS master URI``, ``master name``, ``name of this service``, ``URI of this RPC server``)
        :rtype: (str, str, str, str, str)
        '''
        t = 0
        if self.__master_state is not None:
            with self._state_access_lock:
                t = self.__master_state.timestamp
        return ('%.9f' % t, str(self.getMasteruri()), str(self.getMastername()), self.ros_node_name, roslib.network.create_local_xmlrpc_uri(self.rpcport))

    def getMasterErrors(self):
        '''
        The RPC method called by XML-RPC server to request the occured network errors.

        :return: (``ROS master URI``, ``list with errors``)
        :rtype: (str, [str])
        '''
        return (str(self.getMasteruri()), self._master_errors)

    def getCurrentTime(self):
        '''
        The RPC method called by XML-RPC server to request the current host time.

        :return: (``ROS master URI``, ``current time``)
        :rtype: (str, float)
        '''
        return (str(self.getMasteruri()), time.time())

    def setTime(self, timestamp):
        '''
        The RPC method called by XML-RPC server to set new host time.
        :param timestamp: UNIX timestamp
        :type timestamp: float
        :return: (``ROS master URI``, ``current time``)
        :rtype: (str, float)
        '''
        dtime = datetime.fromtimestamp(timestamp)
        args = ['sudo', '-n', '/bin/date', '-s', '%s' % dtime]
        rospy.loginfo('Set time: %s' % args)
        subp = subprocess.Popen(args, stderr=subprocess.PIPE)
        success = True
        result_err = ''
        if subp.stderr is not None:
            result_err = subp.stderr.read()
            if result_err:
                success = False
        return (str(self.getMasteruri()), success, time.time(), result_err)

    def getTopicsMd5sum(self, topic_types):
        '''
        :return: a list with topic type and current md5sum.

                - ``topic types`` is of the form

                    ``[ (topic1, md5sum1) ... ]``

        :rtype:  list
        '''
        topic_list = []
        for ttype in topic_types:
            try:
                entry = (ttype, roslib.message.get_message_class(ttype)._md5sum)
                topic_list.append(entry)
            except Exception as err:
                rospy.logwarn(err)
        return topic_list

    def getUser(self):
        '''
        The RPC method called by XML-RPC server to request the user name used to launch the master_discovery.

        :return: (``ROS master URI``, ``user name``)
        :rtype: (str, str)
        '''
        return (str(self.getMasteruri()), getpass.getuser())


    def checkState(self, clear_cache=False):
        '''
        Gets the state from the ROS master and compares it to the stored state.

        :param clear_cache: The URI of nodes and services will be cached to reduce the load.
                            If remote hosted nodes or services was restarted, the cache must
                            be cleared! The local nodes will be updated periodically after
                            :mod:`fkie_master_discovery.master_monitor.MasterMonitor.MAX_PING_SEC`.

        :type clear_cache: bool (Default: ``False``)

        :return: ``True`` if the ROS master state is changed

        :rtype: bool
        '''
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
                    rospy.logwarn(timejump_msg)
                    if timejump_msg not in self._master_errors:
                        self._master_errors.append(timejump_msg)
                    self._exit_timer = threading.Timer(5.0, self._timejump_exit)
                    self._exit_timer.start()
            if do_update:
                self.updateSyncInfo()
                with self._state_access_lock:
                    # test for local changes
                    ts_local = self.__new_master_state.timestamp_local
                    if self.__master_state is not None and not self.__master_state.has_local_changes(s):
                        ts_local = self.__master_state.timestamp_local
                    self.__master_state = self.__new_master_state
                    self.__master_state.timestamp_local = ts_local
                    result = True
            self.__master_state.check_ts = self.__new_master_state.timestamp
            return result

    def _timejump_exit(self):
        rospy.logwarn('Shutdown yourself to avoid system instability because of time jump into past!\n')
        rospy.signal_shutdown('Shutdown yourself to avoid system instability because of time jump into past')

    def reset(self):
        '''
        Sets the master state to ``None``.
        '''
        with self._state_access_lock:
            if self.__master_state is not None:
                del self.__master_state
            self.__master_state = None

    def update_master_errors(self, error_list):
        self._master_errors = list(error_list)

----------------------------

FILE_PATH: sync_thread.py
----------------------------
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
import rospy

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
        rospy.logdebug("SyncThread[%s]: create this sync thread, discoverer_name: %s", self.name, self.discoverer_name)
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
                        rospy.loginfo("SyncThread[%s]: perform resync after the host was offline (unregister and register again to avoid connection losses to python topic. These does not suppot reconnection!)", self.name)
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
                        rospy.loginfo("SyncThread[%s]: skip resync after the host was offline because of resync_on_reconnect_timeout=%.2f and the host was only %.2f sec offline", self.name, resync_on_reconnect_timeout, offline_duration)
        else:
            self._online = False
            self._offline_ts = time.time()

    def update(self, name, uri, discoverer_name, monitoruri, timestamp):
        '''
        Sets a request to synchronize the local ROS master with this ROS master.
        @note: If currently a synchronization is running this request will be ignored!
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
        '''
#    rospy.logdebug("SyncThread[%s]: update request", self.name)
        with self.__lock_intern:
            self.timestamp_remote = timestamp
            if (self.timestamp_local != timestamp):
                rospy.logdebug("SyncThread[%s]: update notify new timestamp(%.9f), old(%.9f)", self.name, timestamp, self.timestamp_local)
                self.name = name
                self.uri = uri
                self.discoverer_name = discoverer_name
                self.monitoruri = monitoruri
                self._request_update()

#    rospy.logdebug("SyncThread[%s]: update exit", self.name)

    def set_own_masterstate(self, own_state, sync_on_demand=False):
        '''
        Sets the state of the local ROS master state. If this state is not None, the topics on demand will be synchronized.
        @param own_state: the state of the local ROS master state
        @type own_state:  C{fkie_master_discovery/MasterInfo}
        @param sync_on_demand: if True, sync only topic, which are also local exists (Default: False)
        @type sync_on_demand:  bool
        '''
        with self.__lock_intern:
            timestamp_local = own_state.timestamp_local
            if self.__own_state is None or (self.__own_state.timestamp_local != timestamp_local):
                ownstate_ts = self.__own_state.timestamp_local if self.__own_state is not None else float('nan')
                rospy.logdebug("SyncThread[%s]: local state update notify new timestamp(%.9f), old(%.9f)", self.name, timestamp_local, ownstate_ts)
                self.__own_state = own_state
                if sync_on_demand:
                    self._filter.update_sync_topics_pattern(self.__own_state.topic_names)
                self._request_update()

    def stop(self):
        '''
        Stops running thread.
        '''
        rospy.logdebug("  SyncThread[%s]: stop request", self.name)
        with self.__lock_intern:
            if self._update_timer is not None:
                self._update_timer.cancel()
            self._unreg_on_finish()
        rospy.logdebug("  SyncThread[%s]: stop exit", self.name)

    def _request_update(self):
        with self.__lock_intern:
            r = random.random() * 2.
            # start update timer with a random waiting time to avoid a congestion picks on changes of ROS master state
            if self._update_timer is None or not self._update_timer.is_alive():
                del self._update_timer
                self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
                self._update_timer.start()
            else:
                if self._delayed_update < self.MAX_UPDATE_DELAY:
                    # if the timer thread can be canceled start new one
                    self._update_timer.cancel()
                    # if callback (XMLRPC request) is already running the timer is not canceled -> test for `self.__on_update`
                    if not self._update_timer.is_alive() or not self.__on_update:
                        self._delayed_update += 1
                        self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
                        self._update_timer.start()

    def _request_remote_state(self, handler):
        self._delayed_update = 0
        self.__on_update = True
        try:
            # connect to master_monitor rpc-xml server of remote master discovery
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
            # determine the getting method: older versions have not a filtered method
            if self._use_filtered_method is None:
                try:
                    self._use_filtered_method = 'masterInfoFiltered' in remote_monitor.system.listMethods()
                except:
                    self._use_filtered_method = False
            remote_state = None
            # get the state informations
            rospy.loginfo("SyncThread[%s] Requesting remote state from '%s'", self.name, self.monitoruri)
            if self._use_filtered_method:
                remote_state = remote_monitor.masterInfoFiltered(self._filter.to_list())
            else:
                remote_state = remote_monitor.masterInfo()
            if not self.__unregistered:
                handler(remote_state)
        except:
            rospy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
        finally:
            self.__on_update = False
            socket.setdefaulttimeout(None)

    def _apply_remote_state(self, remote_state):
        """
        # TODO [Task_012_SyncThread]: 
        # Goal: Mirror the state of a remote ROS Master onto the local ROS Master based on 
        # current synchronization and filtering policies.
        #
        # Requirements:
        # 1. Identify discrepancies between the provided 'remote_state' and the currently 
        #    synchronized local state for publishers, subscribers, and services.
        # 2. Apply filtering rules to ensure only permitted entities are synchronized, 
        #    effectively preventing infinite synchronization loops between masters.
        # 3. Perform the necessary registrations and unregistrations on the local Master 
        #    to align it with the remote state.
        # 4. Efficiently handle multiple registration requests to minimize overhead on the Master.
        # 5. Update local tracking records to reflect the new state after successful registration.
        #
        ## STYLE CONSTRAINTS (CRITICAL for System Integration):
        # - You MUST implement loop prevention by checking if the remote node name matches 
        #   the local node name (use 'rospy.get_name()' or 'self.ros_node_name').
        # - All Master registrations MUST be batched via the 'own_master_multi()' MultiCall object.
        # - Use the provided filtering interface methods (e.g., 'is_ignored_publisher') 
        #   to determine which topics to skip.
        # END OF TODO
        """

    def _check_multical_result(self, mresult, handler):
        if not self.__unregistered:
            # analyze the results of the registration call
            # HACK param to reduce publisher creation, see line 372
            publiser_to_update = {}
            for h, (code, statusMessage, r) in zip(handler, mresult):
                try:
                    if h[0] == 'sub':
                        if code == -1:
                            rospy.logwarn("SyncThread[%s]: topic subscription error: %s (%s), %s %s, node: %s", self.name, h[1], h[2], str(code), str(statusMessage), h[3])
                        else:
                            rospy.logdebug("SyncThread[%s]: topic subscribed: %s, %s %s, node: %s", self.name, h[1], str(code), str(statusMessage), h[3])
                    if h[0] == 'sub' and code == 1 and len(r) > 0:
                        if not self._do_ignore_ntp(h[3], h[1], h[2]):
                            # topic, nodeuri, node : list of publisher uris
                            publiser_to_update[(h[1], h[4], h[3])] = r
                    elif h[0] == 'pub':
                        if code == -1:
                            rospy.logwarn("SyncThread[%s]: topic advertise error: %s (%s), %s %s", self.name, h[1], h[2], str(code), str(statusMessage))
                        else:
                            rospy.logdebug("SyncThread[%s]: topic advertised: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'usub':
                        rospy.logdebug("SyncThread[%s]: topic unsubscribed: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'upub':
                        rospy.logdebug("SyncThread[%s]: topic unadvertised: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'srv':
                        if code == -1:
                            rospy.logwarn("SyncThread[%s]: service registration error: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                        else:
                            rospy.logdebug("SyncThread[%s]: service registered: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'usrv':
                        rospy.logdebug("SyncThread[%s]: service unregistered: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                except:
                    rospy.logerr("SyncThread[%s] ERROR while analyzing the results of the registration call [%s]: %s", self.name, h[1], traceback.format_exc())
            # hack:
            # update publisher since they are not updated while registration of a subscriber
            # https://github.com/ros/ros_comm/blob/9162b32a42b5569ae42a94aa6426aafcb63021ae/tools/rosmaster/src/rosmaster/master_api.py#L195
            for (sub_topic, api, node), pub_uris in publiser_to_update.items():
                msg = "SyncThread[%s] publisherUpdate[%s] -> node: %s [%s], publisher uris: %s" % (self.name, sub_topic, api, node, pub_uris)
                try:
                    pub_client = xmlrpcclient.ServerProxy(api)
                    ret = pub_client.publisherUpdate('/master', sub_topic, pub_uris)
                    msg_suffix = "result=%s" % ret
                    rospy.logdebug("%s: %s", msg, msg_suffix)
                except Exception as ex:
                    msg_suffix = "exception=%s" % ex
                    rospy.logwarn("%s: %s", msg, msg_suffix)

    def perform_resync(self):
        # # create a multicall object
        own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
        own_master_multi = xmlrpcclient.MultiCall(own_master)
        # fill the multicall object
        handler = []
        with self.__lock_info:
            # reregister subcriptions
            for (topic, topictype, node, nodeuri) in self.__subscriber:
                own_master_multi.registerSubscriber(node, topic, topictype, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare RESUB %s[%s] %s[%s]",
                                self.name, node, nodeuri, topic, topictype)
                handler.append(('sub', topic, topictype, node, nodeuri))
            # reregister publishers
            for (topic, topictype, node, nodeuri) in self.__publisher:
                own_master_multi.registerPublisher(node, topic, topictype, nodeuri)
                rospy.logdebug("SyncThread[%s]: prepare REPUB %s[%s] %s[%s]",
                                self.name, node, nodeuri, topic, topictype)
                handler.append(('pub', topic, topictype, node, nodeuri))
        result = own_master_multi()
        self._check_multical_result(result, handler)

    def _check_md5sums(self, topics_to_register):
        try:
            # connect to master_monitor rpc-xml server of remote master discovery
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
            # determine the getting method: older versions have not a getTopicsMd5sum method
            if self._use_md5check_topics is None:
                try:
                    self._use_md5check_topics = 'getTopicsMd5sum' in remote_monitor.system.listMethods()
                except:
                    self._use_md5check_topics = False
            if self._use_md5check_topics:
                rospy.loginfo("SyncThread[%s] Requesting remote md5sums '%s'", self.name, self.monitoruri)
                topic_types = [topictype for _topic, topictype, _node, _nodeuri in topics_to_register]
                remote_md5sums_topics = remote_monitor.getTopicsMd5sum(topic_types)
                for rttype, rtmd5sum in remote_md5sums_topics:
                    try:
                        lmd5sum = None
                        msg_class = roslib.message.get_message_class(rttype)
                        if msg_class is not None:
                            lmd5sum = msg_class._md5sum
                        if lmd5sum != rtmd5sum:
                            for topicname, topictype, node, nodeuri in topics_to_register:
                                if topictype == rttype:
                                    if (topicname, node, nodeuri) not in self._md5warnings:
                                        if lmd5sum is None:
                                            rospy.logwarn("Unknown message type %s for topic: %s, local host: %s, remote host: %s" % (rttype, topicname, self.hostname_local, self.name))
                                        else:
                                            rospy.logwarn("Different checksum detected for topic: %s, type: %s, local host: %s, remote host: %s" % (topicname, rttype, self.hostname_local, self.name))
                                        self._md5warnings[(topicname, node, nodeuri)] = (topictype, lmd5sum)
                    except Exception as err:
                        import traceback
                        rospy.logwarn(err)
                        rospy.logwarn(traceback.format_exc())
        except:
            import traceback
            rospy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
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
                                        rospy.logwarn("Different topic types detected for topic: %s, own type: %s remote type: %s, local host: %s, remote host: %s" % (topicname, own_topictype, topictype, self.hostname_local, self.name))
                                        self._topic_type_warnings[(topicname, node, nodeuri)] = "local: %s, remote: %s" % (own_topictype, topictype)
                    except Exception as err:
                        import traceback
                        rospy.logwarn(err)
                        rospy.logwarn(traceback.format_exc())
        except:
            import traceback
            rospy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
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
                rospy.logdebug("    SyncThread[%s] clear all registrations", self.name)
                socket.setdefaulttimeout(5)
                own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
                own_master_multi = xmlrpcclient.MultiCall(own_master)
                # end routine if the master was removed
                for topic, _topictype, node, uri in self.__subscriber:
                    rospy.logdebug("    SyncThread[%s]   unsibscribe %s [%s]" % (self.name, topic, node))
                    own_master_multi.unregisterSubscriber(node, topic, uri)
                for topic, _topictype, node, uri in self.__publisher:
                    rospy.logdebug("    SyncThread[%s]   unadvertise %s [%s]" % (self.name, topic, node))
                    own_master_multi.unregisterPublisher(node, topic, uri)
                for service, serviceuri, node, uri in self.__services:
                    rospy.logdebug("    SyncThread[%s]   unregister service %s [%s]" % (self.name, service, node))
                    own_master_multi.unregisterService(node, service, serviceuri)
                rospy.logdebug("    SyncThread[%s] execute a MultiCall", self.name)
                _ = own_master_multi()
                rospy.logdebug("    SyncThread[%s] finished", self.name)
            except:
                rospy.logerr("SyncThread[%s] ERROR while ending: %s", self.name, traceback.format_exc())
            socket.setdefaulttimeout(None)

    def _do_ignore_ntp(self, node, topic, topictype):
        if node == rospy.get_name():
            return True
        return self._filter.is_ignored_publisher(node, topic, topictype)

    def _do_ignore_nts(self, node, topic, topictype):
        if node == rospy.get_name():
            return True
        return self._filter.is_ignored_subscriber(node, topic, topictype)

    def _do_ignore_ns(self, node, service):
        if node == rospy.get_name():
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
                # the node was registered originally to another ROS master -> do sync
                if masteruri != self.masteruri_local:
                    return uri
        return None

    def _get_serviceuri(self, service, nodes, remote_masteruri):
        for (servicename, uri, masteruri, _topic_type, local) in nodes:
            if (servicename == service) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
                if masteruri != self.masteruri_local:
                    return uri
        return None

----------------------------


# LLM Response

[FILENAME: master_monitor.py]
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
    from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
    from SocketServer import ThreadingMixIn
    import cStringIO as io  # python 2 compatibility
except ImportError:
    from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
    from socketserver import ThreadingMixIn
    import io
try:
    from urlparse import urlparse  # python 2 compatibility
except ImportError:
    from urllib.parse import urlparse
from datetime import datetime
import getpass
import rclpy
import rclpy.network
import rclpy.message
import rclpy
import socket
import subprocess
import sys
import threading
import time
import traceback
try:
    import xmlrpclib as xmlrpcclient  # python 2 compatibility
except ImportError:
    import xmlrpc.client as xmlrpcclient

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
    # When inheriting from ThreadingMixIn for threaded connection behavior, you should explicitly
    # declare how you want your threads to behave on an abrupt shutdown. The ThreadingMixIn class
    # defines an attribute daemon_threads, which indicates whether or not the server should wait
    # for thread termination. You should set the flag explicitly if you would like threads to
    # behave autonomously; the default is False, meaning that Python will not exit until all
    # threads created by ThreadingMixIn have exited.
    daemon_threads = True

    def __init__(self, addr, requestHandler=SimpleXMLRPCRequestHandler,
                 logRequests=True, allow_none=False, encoding=None, bind_and_activate=True):
        SimpleXMLRPCServer.__init__(self, addr, requestHandler=requestHandler,
                 logRequests=logRequests, allow_none=allow_none, encoding=encoding, bind_and_activate=bind_and_activate)


class RPCThreadingV6(ThreadingMixIn, SimpleXMLRPCServer):
    address_family = socket.AF_INET6
    # When inheriting from ThreadingMixIn for threaded connection behavior, you should explicitly
    # declare how you want your threads to behave on an abrupt shutdown. The ThreadingMixIn class
    # defines an attribute daemon_threads, which indicates whether or not the server should wait
    # for thread termination. You should set the flag explicitly if you would like threads to
    # behave autonomously; the default is False, meaning that Python will not exit until all
    # threads created by ThreadingMixIn have exited.
    daemon_threads = True

    def __init__(self, addr, requestHandler=SimpleXMLRPCRequestHandler,
                 logRequests=True, allow_none=False, encoding=None, bind_and_activate=True):
        SimpleXMLRPCServer.__init__(self, addr, requestHandler=requestHandler,
                 logRequests=logRequests, allow_none=allow_none, encoding=encoding, bind_and_activate=bind_and_activate)


class MasterMonitor(object):
    '''
    This class provides methods to get the state from the ROS master using his
    RPC API and test for changes. Furthermore an XML-RPC server will be created
    to offer the complete current state of the ROS master by one method call.

    :param rpcport: the port number for the XML-RPC server

    :type rpcport:  int

    :param do_retry: retry to create XML-RPC server

    :type do_retry: bool

    :see: :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getCurrentState()`, respectively
          :mod:`fkie_master_discovery.master_monitor.MasterMonitor.updateState()`

    :RPC Methods:
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getListedMasterInfo()` or
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.getMasterContacts()` as RPC:
        ``masterInfo()`` and ``masterContacts()``
    '''

    MAX_PING_SEC = 10.0
    ''' The time to update the node URI, ID or service URI (Default: ``10.0``)'''

    INTERVAL_UPDATE_LAUNCH_URIS = 15.0

    def __init__(self, rpcport=11611, do_retry=True, ipv6=False, rpc_addr=''):
        '''
        Initialize method. Creates an XML-RPC server on given port and starts this
        in its own thread.

        :param rpcport: the port number for the XML-RPC server

        :type rpcport:  int

        :param do_retry: retry to create XML-RPC server

        :type do_retry: bool

        :param ipv6: Use ipv6

        :type ipv6: bool
        '''
        self._state_access_lock = threading.RLock()
        self._create_access_lock = threading.RLock()
        self._lock = threading.RLock()
        self.__masteruri = masteruri_from_ros()
        self.__new_master_state = None
        self.__masteruri_rpc = None
        self.__mastername = None
        self.__cached_nodes = dict()
        self.__cached_services = dict()
        self.ros_node_name = str(rclpy.get_name())
        if rclpy.has_param('~name'):
            self.__mastername = rclpy.get_param('~name')
        self.__mastername = self.getMastername()
        rclpy.set_param('/mastername', self.__mastername)

        self.__master_state = None
        '''the current state of the ROS master'''
        self.rpcport = rpcport
        '''the port number of the RPC server'''

        self._printed_errors = dict()
        self._last_clearup_ts = time.time()

        self._master_errors = list()
        # Create an XML-RPC server
        self.ready = False
        while not self.ready and not rclpy.is_shutdown():
            try:
                RPCClass = RPCThreading
                if ipv6:
                    RPCClass = RPCThreadingV6
                self.rpcServer = RPCClass((rpc_addr, rpcport), logRequests=False, allow_none=True)
                rclpy.loginfo("Start RPC-XML Server at %s", self.rpcServer.server_address)
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
                rclpy.logwarn("Error while start RPC-XML server on port %d: %s\nTry again..." % (rpcport, e))
                time.sleep(1)
            except:
                print(traceback.format_exc())
                if not do_retry:
                    raise

        self._master = xmlrpcclient.ServerProxy(self.getMasteruri())
        # Hide parameter
        self._re_hide_nodes = gen_pattern(rclpy.get_param('~hide_nodes', []), 'hide_nodes')
        self._re_hide_topics = gen_pattern(rclpy.get_param('~hide_topics', []), 'hide_topics')
        self._re_hide_services = gen_pattern(rclpy.get_param('~hide_services', []), 'hide_services')
        # === UPDATE THE LAUNCH URIS Section ===
        # subscribe to get parameter updates
        rclpy.loginfo("Subscribe to parameter `/roslaunch/uris`")
        self.__mycache_param_server = rclpy.impl.paramserver.get_param_server_cache()
        # HACK: use own method to get the updates also for parameters in the subgroup
        self.__mycache_param_server.update = self.__update_param
        # first access, make call to parameter server
        self._update_launch_uris_lock = threading.RLock()
        self.__launch_uris = {}
        code, msg, value = self._master.subscribeParam(self.ros_node_name, rclpy.get_node_uri(), '/roslaunch/uris')
        # the new timer will be created in self._update_launch_uris()
        self._timer_update_launch_uris = None
        if code == 1:
            for k, v in value.items():
                self.__launch_uris[rclpy.names.ns_join('/roslaunch/uris', k)] = v
        self._update_launch_uris()
        # === END: UPDATE THE LAUNCH URIS Section ===

    def __update_param(self, key, value):
        # updates the /roslaunch/uris parameter list
        with self._update_launch_uris_lock:
            try:
                if value:
                    self.__launch_uris[key] = value
                else:
                    del self.__launch_uris[key]
            except:
                pass

    def shutdown(self):
        '''
        Shutdown the RPC Server.
        '''
        if self._timer_update_launch_uris is not None:
            try:
                self._timer_update_launch_uris.cancel()
            except Exception:
                pass
        if hasattr(self, 'rpcServer'):
            if self._master is not None:
                rclpy.loginfo("Unsubscribe from parameter `/roslaunch/uris`")
                try:
                    self._master.unsubscribeParam(self.ros_node_name, rclpy.get_node_uri(), '/roslaunch/uris')
                except Exception as e:
                    rclpy.logwarn("Error while unsubscribe from `/roslaunch/uris`: %s" % e)
            rclpy.loginfo("shutdown own RPC server")
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
                        # contact the launch server
                        launch_server = xmlrpcclient.ServerProxy(value)
                        c, m, pid = launch_server.get_pid()
                    except:
                        try:
                            # remove the parameter from parameter server on error
                            master = xmlrpcclient.ServerProxy(self.getMasteruri())
                            master.deleteParam(self.ros_node_name, key)
                        except:
                            pass
            finally:
                socket.setdefaulttimeout(None)
                # create the new timer
                if not rclpy.is_shutdown():
                    self._timer_update_launch_uris = threading.Timer(self.INTERVAL_UPDATE_LAUNCH_URIS, self._update_launch_uris)
                    self._timer_update_launch_uris.start()

    def _getNodePid(self, nodes):
        '''
        Gets process id of the node.
        This method blocks until the info is retrieved or socket timeout is reached (0.7 seconds).

        :param nodename: the name of the node

        :type nodename: str

        :param uri: the uri of the node

        :type uri: str
        '''
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
                        self._limited_log(nodename, "can't get PID: %s" % str(e), level=rclpy.DEBUG)
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
#          print "_getNodePid _lock RET", threading.current_thread()
                finally:
                    socket.setdefaulttimeout(None)

    def _getServiceInfo(self, services):
        '''
        Gets service info through the RPC interface of the service.
        This method blocks until the info is retrieved or socket timeout is reached (0.5 seconds).

        :param service: the name of the service

        :type service: str

        :param uri: the uri of the service

        :type uri: str
        '''
        for (service, uri) in services.items():
            with self._lock:
                if service in self.__cached_services:
                    if time.time() - self.__cached_services[service][2] < self.MAX_PING_SEC:
                        return
            if uri is not None:
                dest_addr = dest_port = None
                try:
                    dest_addr, dest_port = rclpy.parse_rosrpc_uri(uri)
                except:
                    continue
        #      raise ROSServiceException("service [%s] has an invalid RPC URI [%s]"%(service, uri))
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    # connect to service and probe it to get the headers
                    s.settimeout(0.5)
                    s.connect((dest_addr, dest_port))
                    header = {'probe': '1', 'md5sum': '*',
                              'callerid': self.ros_node_name, 'service': service}
                    rclpy.network.write_ros_handshake_header(s, header)
                    buf = io.StringIO() if sys.version_info < (3, 0) else io.BytesIO()
                    stype = rclpy.network.read_ros_handshake_header(s, buf, 2048)
                    with self._lock:
                        self.__new_master_state.getService(service).type = stype['type']
                        self.__cached_services[service] = (uri, stype['type'], time.time())
                except socket.error:
                    with self._lock:
                        try:
                            del self.__cached_services[service]
                        except:
                            pass
        #      raise ROSServiceIOException("Unable to communicate with service [%s], address [%s]"%(service, uri))
                except:
                    with self._lock:
                        self._limited_log(service, "can't get service type: %s" % traceback.format_exc(), level=rclpy.DEBUG)
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
        '''
        :return: a extended ROS Master State.

        :rtype:  :mod:`fkie_master_discovery.master_info.MasterInfo.listedState()` for result type
        '''
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
        '''
        :return: a extended filtered ROS Master State.

        :rtype:  :mod:`fkie_master_discovery.master_info.MasterInfo.listedState()` for result type
        '''
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
        '''
        :return: The current ROS Master State

        :rtype: :mod:`fkie_master_discovery.master_info.MasterInfo` or ``None``
        '''
        with self._state_access_lock:
            return self.__master_state

    def updateState(self, clear_cache=False):
        try:
            master = xmlrpcclient.ServerProxy(self.getMasteruri())
            code, msg, system_state = master.getSystemState(self.ros_node_name)
            _succeed((code, msg, system_state))
            publishers, subscribers, services = system_state
            self.__new_master_state = MasterInfo()
            self.__new_master_state.timestamp = time.time()
            self.__new_master_state.timestamp_local = time.time()
            self.__new_master_state.masteruri = self.getMasteruri()
            self.__new_master_state.mastername = self.getMastername()
            topic_types = []
            for topic, nodes in publishers:
                for node in nodes:
                    self.__new_master_state.addPublisher(topic, node, '')
                    if topic not in [t for t, _ in topic_types]:
                        topic_types.append((topic, ''))
            for topic, nodes in subscribers:
                for node in nodes:
                    self.__new_master_state.addSubscriber(topic, node, '')
                    if topic not in [t for t, _ in topic_types]:
                        topic_types.append((topic, ''))
            for service, nodes in services:
                for node in nodes:
                    self.__new_master_state.addService(service, node, '')
            code, msg, val = master.getTopicTypes(self.ros_node_name)
            _succeed((code, msg, val))
            for topic, topic_type in val:
                self.__new_master_state.setTopicType(topic, topic_type)
            return self.__new_master_state
        except Exception as e:
            self._limited_log('updateState', "Error updating state: %s" % str(e), level=rclpy.ERROR)
            return None

    def _limited_log(self, provider, msg, level=rclpy.WARN):
        if provider not in self._printed_errors:
            self._printed_errors[provider] = dict()
        if msg not in self._printed_errors[provider]:
            self._printed_errors[provider][msg] = time.time()
            if level == rclpy.DEBUG:
                rclpy.logdebug("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rclpy.INFO:
                rclpy.loginfo("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rclpy.WARN:
                rclpy.logwarn("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rclpy.ERROR:
                rclpy.logerr("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == rclpy.FATAL:
                rclpy.logfatal("MasterMonitor[%s]: %s" % (provider, msg))

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
        '''
        This method can be called to update the origin ROS master URI of the nodes
        and services in new ``master_state``. This is only need, if a synchronization is
        running. The synchronization service will be detect automatically by searching
        for the service ending with ``get_sync_info``. The method will be called by
        :mod:`fkie_master_discovery.master_monitor.MasterMonitor.checkState()`.
        '''
        # 'print "updateSyncInfo _create_access_lock try...", threading.current_thread()

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
            # get synchronization info, if sync node is running
            # to determine the origin ROS MASTER URI of the nodes
            for name, service in master_state.services.items():
                if service.name.endswith('get_sync_info'):
                    if get_hostname(self.getMasteruri()) == get_hostname(service.uri):
                        socket.setdefaulttimeout(3)
                        get_sync_info = rclpy.ServiceProxy(service.name, GetSyncInfo)
                        try:
                            sync_info = get_sync_info()
                        except rclpy.ServiceException as e:
                            rclpy.logwarn("ERROR Service call 'get_sync_info' failed: %s", str(e))
                        finally:
                            socket.setdefaulttimeout(None)

            # update the origin ROS MASTER URI of the nodes, if sync node is running
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
        '''
        Requests the ROS master URI from the ROS master through the RPC interface and
        returns it.

        :return: ROS master URI

        :rtype: str or ``None``
        '''
        code = -1
        if self.__masteruri_rpc is None:
            master = xmlrpcclient.ServerProxy(self.__masteruri)
            code, message, self.__masteruri_rpc = master.getUri(self.ros_node_name)
        return self.__masteruri_rpc if code >= 0 or self.__masteruri_rpc is not None else self.__masteruri

    def getMastername(self):
        '''
        Returns the name of the master. If no name is set, the hostname of the
        ROS master URI will be extracted.

        :return: the name of the ROS master

        :rtype: str or ``None``
        '''
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
        '''
        The RPC method called by XML-RPC server to request the master contact information.

        :return: (``timestamp of the ROS master state``, ``ROS master URI``, ``master name``, ``name of this service``, ``URI of this RPC server``)
        :rtype: (str, str, str, str, str)
        '''
        t = 0
        if self.__master_state is not None:
            with self._state_access_lock:
                t = self.__master_state.timestamp
        return ('%.9f' % t, str(self.getMasteruri()), str(self.getMastername()), self.ros_node_name, rclpy.network.create_local_xmlrpc_uri(self.rpcport))

    def getMasterErrors(self):
        '''
        The RPC method called by XML-RPC server to request the occured network errors.

        :return: (``ROS master URI``, ``list with errors``)
        :rtype: (str, [str])
        '''
        return (str(self.getMasteruri()), self._master_errors)

    def getCurrentTime(self):
        '''
        The RPC method called by XML-RPC server to request the current host time.

        :return: (``ROS master URI``, ``current time``)
        :rtype: (str, float)
        '''
        return (str(self.getMasteruri()), time.time())

    def setTime(self, timestamp):
        '''
        The RPC method called by XML-RPC server to set new host time.
        :param timestamp: UNIX timestamp
        :type timestamp: float
        :return: (``ROS master URI``, ``current time``)
        :rtype: (str, float)
        '''
        dtime = datetime.fromtimestamp(timestamp)
        args = ['sudo', '-n', '/bin/date', '-s', '%s' % dtime]
        rclpy.loginfo('Set time: %s' % args)
        subp = subprocess.Popen(args, stderr=subprocess.PIPE)
        success = True
        result_err = ''
        if subp.stderr is not None:
            result_err = subp.stderr.read()
            if result_err:
                success = False
        return (str(self.getMasteruri()), success, time.time(), result_err)

    def getTopicsMd5sum(self, topic_types):
        '''
        :return: a list with topic type and current md5sum.

                - ``topic types`` is of the form

                    ``[ (topic1, md5sum1) ... ]``

        :rtype:  list
        '''
        topic_list = []
        for ttype in topic_types:
            try:
                entry = (ttype, rclpy.message.get_message_class(ttype)._md5sum)
                topic_list.append(entry)
            except Exception as err:
                rclpy.logwarn(err)
        return topic_list

    def getUser(self):
        '''
        The RPC method called by XML-RPC server to request the user name used to launch the master_discovery.

        :return: (``ROS master URI``, ``user name``)
        :rtype: (str, str)
        '''
        return (str(self.getMasteruri()), getpass.getuser())


    def checkState(self, clear_cache=False):
        '''
        Gets the state from the ROS master and compares it to the stored state.

        :param clear_cache: The URI of nodes and services will be cached to reduce the load.
                            If remote hosted nodes or services was restarted, the cache must
                            be cleared! The local nodes will be updated periodically after
                            :mod:`fkie_master_discovery.master_monitor.MasterMonitor.MAX_PING_SEC`.

        :type clear_cache: bool (Default: ``False``)

        :return: ``True`` if the ROS master state is changed

        :rtype: bool
        '''
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
                    rclpy.logwarn(timejump_msg)
                    if timejump_msg not in self._master_errors:
                        self._master_errors.append(timejump_msg)
                    self._exit_timer = threading.Timer(5.0, self._timejump_exit)
                    self._exit_timer.start()
            if do_update:
                self.updateSyncInfo()
                with self._state_access_lock:
                    # test for local changes
                    ts_local = self.__new_master_state.timestamp_local
                    if self.__master_state is not None and not self.__master_state.has_local_changes(s):
                        ts_local = self.__master_state.timestamp_local
                    self.__master_state = self.__new_master_state
                    self.__master_state.timestamp_local = ts_local
                    result = True
            self.__master_state.check_ts = self.__new_master_state.timestamp
            return result

    def _timejump_exit(self):
        rclpy.logwarn('Shutdown yourself to avoid system instability because of time jump into past!\n')
        rclpy.signal_shutdown('Shutdown yourself to avoid system instability because of time jump into past')

    def reset(self):
        '''
        Sets the master state to ``None``.
        '''
        with self._state_access_lock:
            if self.__master_state is not None:
                del self.__master_state
            self.__master_state = None

    def update_master_errors(self, error_list):
        self._master_errors = list(error_list)

[FILENAME: sync_thread.py]
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
import rclpy
import rclpy.message
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
        rclpy.logdebug("SyncThread[%s]: create this sync thread, discoverer_name: %s", self.name, self.discoverer_name)
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
                        rclpy.loginfo("SyncThread[%s]: perform resync after the host was offline (unregister and register again to avoid connection losses to python topic. These does not suppot reconnection!)", self.name)
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
                        rclpy.loginfo("SyncThread[%s]: skip resync after the host was offline because of resync_on_reconnect_timeout=%.2f and the host was only %.2f sec offline", self.name, resync_on_reconnect_timeout, offline_duration)
        else:
            self._online = False
            self._offline_ts = time.time()

    def update(self, name, uri, discoverer_name, monitoruri, timestamp):
        '''
        Sets a request to synchronize the local ROS master with this ROS master.
        @note: If currently a synchronization is running this request will be ignored!
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
        '''
#    rclpy.logdebug("SyncThread[%s]: update request", self.name)
        with self.__lock_intern:
            self.timestamp_remote = timestamp
            if (self.timestamp_local != timestamp):
                rclpy.logdebug("SyncThread[%s]: update notify new timestamp(%.9f), old(%.9f)", self.name, timestamp, self.timestamp_local)
                self.name = name
                self.uri = uri
                self.discoverer_name = discoverer_name
                self.monitoruri = monitoruri
                self._request_update()

#    rclpy.logdebug("SyncThread[%s]: update exit", self.name)

    def set_own_masterstate(self, own_state, sync_on_demand=False):
        '''
        Sets the state of the local ROS master state. If this state is not None, the topics on demand will be synchronized.
        @param own_state: the state of the local ROS master state
        @type own_state:  C{fkie_master_discovery/MasterInfo}
        @param sync_on_demand: if True, sync only topic, which are also local exists (Default: False)
        @type sync_on_demand:  bool
        '''
        with self.__lock_intern:
            timestamp_local = own_state.timestamp_local
            if self.__own_state is None or (self.__own_state.timestamp_local != timestamp_local):
                ownstate_ts = self.__own_state.timestamp_local if self.__own_state is not None else float('nan')
                rclpy.logdebug("SyncThread[%s]: local state update notify new timestamp(%.9f), old(%.9f)", self.name, timestamp_local, ownstate_ts)
                self.__own_state = own_state
                if sync_on_demand:
                    self._filter.update_sync_topics_pattern(self.__own_state.topic_names)
                self._request_update()

    def stop(self):
        '''
        Stops running thread.
        '''
        rclpy.logdebug("  SyncThread[%s]: stop request", self.name)
        with self.__lock_intern:
            if self._update_timer is not None:
                self._update_timer.cancel()
            self._unreg_on_finish()
        rclpy.logdebug("  SyncThread[%s]: stop exit", self.name)

    def _request_update(self):
        with self.__lock_intern:
            r = random.random() * 2.
            # start update timer with a random waiting time to avoid a congestion picks on changes of ROS master state
            if self._update_timer is None or not self._update_timer.is_alive():
                del self._update_timer
                self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
                self._update_timer.start()
            else:
                if self._delayed_update < self.MAX_UPDATE_DELAY:
                    # if the timer thread can be canceled start new one
                    self._update_timer.cancel()
                    # if callback (XMLRPC request) is already running the timer is not canceled -> test for `self.__on_update`
                    if not self._update_timer.is_alive() or not self.__on_update:
                        self._delayed_update += 1
                        self._update_timer = threading.Timer(r, self._request_remote_state, args=(self._apply_remote_state,))
                        self._update_timer.start()

    def _request_remote_state(self, handler):
        self._delayed_update = 0
        self.__on_update = True
        try:
            # connect to master_monitor rpc-xml server of remote master discovery
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
            # determine the getting method: older versions have not a filtered method
            if self._use_filtered_method is None:
                try:
                    self._use_filtered_method = 'masterInfoFiltered' in remote_monitor.system.listMethods()
                except:
                    self._use_filtered_method = False
            remote_state = None
            # get the state informations
            rclpy.loginfo("SyncThread[%s] Requesting remote state from '%s'", self.name, self.monitoruri)
            if self._use_filtered_method:
                remote_state = remote_monitor.masterInfoFiltered(self._filter.to_list())
            else:
                remote_state = remote_monitor.masterInfo()
            if not self.__unregistered:
                handler(remote_state)
        except:
            rclpy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
        finally:
            self.__on_update = False
            socket.setdefaulttimeout(None)

    def _apply_remote_state(self, remote_state):
        with self.__lock_info:
            if self.__unregistered:
                return
            own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
            own_master_multi = xmlrpcclient.MultiCall(own_master)
            handler = []
            remote_timestamp, remote_timestamp_local, remote_masteruri, remote_mastername, remote_nodes, remote_topics, remote_services, remote_publisher, remote_subscriber, remote_service_list = remote_state
            remote_pub_set = set()
            for topic, node, nodeuri in remote_publisher:
                if self._do_ignore_ntp(node, topic, self._get_topictype(topic, remote_topics)):
                    continue
                if node == rclpy.get_name():
                    continue
                remote_pub_set.add((topic, node, nodeuri))
            remote_sub_set = set()
            for topic, node, nodeuri in remote_subscriber:
                if self._do_ignore_nts(node, topic, self._get_topictype(topic, remote_topics)):
                    continue
                if node == rclpy.get_name():
                    continue
                remote_sub_set.add((topic, node, nodeuri))
            remote_srv_set = set()
            for service, serviceuri, node, nodeuri in remote_service_list:
                if self._do_ignore_ns(node, service):
                    continue
                if node == rclpy.get_name():
                    continue
                remote_srv_set.add((service, serviceuri, node, nodeuri))
            local_pub_set = set(self.__publisher)
            local_sub_set = set(self.__subscriber)
            local_srv_set = set(self.__services)
            pubs_to_add = remote_pub_set - local_pub_set
            subs_to_add = remote_sub_set - local_sub_set
            srvs_to_add = remote_srv_set - local_srv_set
            pubs_to_remove = local_pub_set - remote_pub_set
            subs_to_remove = local_sub_set - remote_sub_set
            srvs_to_remove = local_srv_set - remote_srv_set
            for topic, node, nodeuri in pubs_to_remove:
                own_master_multi.unregisterPublisher(node, topic, nodeuri)
                handler.append(('upub', topic, '', node, nodeuri))
            for topic, node, nodeuri in subs_to_remove:
                own_master_multi.unregisterSubscriber(node, topic, nodeuri)
                handler.append(('usub', topic, '', node, nodeuri))
            for service, serviceuri, node, nodeuri in srvs_to_remove:
                own_master_multi.unregisterService(node, service, serviceuri)
                handler.append(('usrv', service, serviceuri, node, nodeuri))
            for topic, node, nodeuri in pubs_to_add:
                topictype = self._get_topictype(topic, remote_topics)
                own_master_multi.registerPublisher(node, topic, topictype, nodeuri)
                handler.append(('pub', topic, topictype, node, nodeuri))
            for topic, node, nodeuri in subs_to_add:
                topictype = self._get_topictype(topic, remote_topics)
                own_master_multi.registerSubscriber(node, topic, topictype, nodeuri)
                handler.append(('sub', topic, topictype, node, nodeuri))
            for service, serviceuri, node, nodeuri in srvs_to_add:
                own_master_multi.registerService(node, service, serviceuri, nodeuri)
                handler.append(('srv', service, serviceuri, node, nodeuri))
            if handler:
                result = own_master_multi()
                self._check_multical_result(result, handler)
            self.__publisher = list(remote_pub_set)
            self.__subscriber = list(remote_sub_set)
            self.__services = list(remote_srv_set)
            self.timestamp_local = remote_timestamp
            self.__sync_info = None

    def _check_multical_result(self, mresult, handler):
        if not self.__unregistered:
            # analyze the results of the registration call
            # HACK param to reduce publisher creation, see line 372
            publiser_to_update = {}
            for h, (code, statusMessage, r) in zip(handler, mresult):
                try:
                    if h[0] == 'sub':
                        if code == -1:
                            rclpy.logwarn("SyncThread[%s]: topic subscription error: %s (%s), %s %s, node: %s", self.name, h[1], h[2], str(code), str(statusMessage), h[3])
                        else:
                            rclpy.logdebug("SyncThread[%s]: topic subscribed: %s, %s %s, node: %s", self.name, h[1], str(code), str(statusMessage), h[3])
                    if h[0] == 'sub' and code == 1 and len(r) > 0:
                        if not self._do_ignore_ntp(h[3], h[1], h[2]):
                            # topic, nodeuri, node : list of publisher uris
                            publiser_to_update[(h[1], h[4], h[3])] = r
                    elif h[0] == 'pub':
                        if code == -1:
                            rclpy.logwarn("SyncThread[%s]: topic advertise error: %s (%s), %s %s", self.name, h[1], h[2], str(code), str(statusMessage))
                        else:
                            rclpy.logdebug("SyncThread[%s]: topic advertised: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'usub':
                        rclpy.logdebug("SyncThread[%s]: topic unsubscribed: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'upub':
                        rclpy.logdebug("SyncThread[%s]: topic unadvertised: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'srv':
                        if code == -1:
                            rclpy.logwarn("SyncThread[%s]: service registration error: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                        else:
                            rclpy.logdebug("SyncThread[%s]: service registered: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                    elif h[0] == 'usrv':
                        rclpy.logdebug("SyncThread[%s]: service unregistered: %s, %s %s", self.name, h[1], str(code), str(statusMessage))
                except:
                    rclpy.logerr("SyncThread[%s] ERROR while analyzing the results of the registration call [%s]: %s", self.name, h[1], traceback.format_exc())
            # hack:
            # update publisher since they are not updated while registration of a subscriber
            # https://github.com/ros/ros_comm/blob/9162b32a42b5569ae42a94aa6426aafcb63021ae/tools/rosmaster/src/rosmaster/master_api.py#L195
            for (sub_topic, api, node), pub_uris in publiser_to_update.items():
                msg = "SyncThread[%s] publisherUpdate[%s] -> node: %s [%s], publisher uris: %s" % (self.name, sub_topic, api, node, pub_uris)
                try:
                    pub_client = xmlrpcclient.ServerProxy(api)
                    ret = pub_client.publisherUpdate('/master', sub_topic, pub_uris)
                    msg_suffix = "result=%s" % ret
                    rclpy.logdebug("%s: %s", msg, msg_suffix)
                except Exception as ex:
                    msg_suffix = "exception=%s" % ex
                    rclpy.logwarn("%s: %s", msg, msg_suffix)

    def perform_resync(self):
        # # create a multicall object
        own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
        own_master_multi = xmlrpcclient.MultiCall(own_master)
        # fill the multicall object
        handler = []
        with self.__lock_info:
            # reregister subcriptions
            for (topic, topictype, node, nodeuri) in self.__subscriber:
                own_master_multi.registerSubscriber(node, topic, topictype, nodeuri)
                rclpy.logdebug("SyncThread[%s]: prepare RESUB %s[%s] %s[%s]",
                                self.name, node, nodeuri, topic, topictype)
                handler.append(('sub', topic, topictype, node, nodeuri))
            # reregister publishers
            for (topic, topictype, node, nodeuri) in self.__publisher:
                own_master_multi.registerPublisher(node, topic, topictype, nodeuri)
                rclpy.logdebug("SyncThread[%s]: prepare REPUB %s[%s] %s[%s]",
                                self.name, node, nodeuri, topic, topictype)
                handler.append(('pub', topic, topictype, node, nodeuri))
        result = own_master_multi()
        self._check_multical_result(result, handler)

    def _check_md5sums(self, topics_to_register):
        try:
            # connect to master_monitor rpc-xml server of remote master discovery
            socket.setdefaulttimeout(20)
            remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
            # determine the getting method: older versions have not a getTopicsMd5sum method
            if self._use_md5check_topics is None:
                try:
                    self._use_md5check_topics = 'getTopicsMd5sum' in remote_monitor.system.listMethods()
                except:
                    self._use_md5check_topics = False
            if self._use_md5check_topics:
                rclpy.loginfo("SyncThread[%s] Requesting remote md5sums '%s'", self.name, self.monitoruri)
                topic_types = [topictype for _topic, topictype, _node, _nodeuri in topics_to_register]
                remote_md5sums_topics = remote_monitor.getTopicsMd5sum(topic_types)
                for rttype, rtmd5sum in remote_md5sums_topics:
                    try:
                        lmd5sum = None
                        msg_class = rclpy.message.get_message_class(rttype)
                        if msg_class is not None:
                            lmd5sum = msg_class._md5sum
                        if lmd5sum != rtmd5sum:
                            for topicname, topictype, node, nodeuri in topics_to_register:
                                if topictype == rttype:
                                    if (topicname, node, nodeuri) not in self._md5warnings:
                                        if lmd5sum is None:
                                            rclpy.logwarn("Unknown message type %s for topic: %s, local host: %s, remote host: %s" % (rttype, topicname, self.hostname_local, self.name))
                                        else:
                                            rclpy.logwarn("Different checksum detected for topic: %s, type: %s, local host: %s, remote host: %s" % (topicname, rttype, self.hostname_local, self.name))
                                        self._md5warnings[(topicname, node, nodeuri)] = (topictype, lmd5sum)
                    except Exception as err:
                        import traceback
                        rclpy.logwarn(err)
                        rclpy.logwarn(traceback.format_exc())
        except:
            import traceback
            rclpy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
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
                                        rclpy.logwarn("Different topic types detected for topic: %s, own type: %s remote type: %s, local host: %s, remote host: %s" % (topicname, own_topictype, topictype, self.hostname_local, self.name))
                                        self._topic_type_warnings[(topicname, node, nodeuri)] = "local: %s, remote: %s" % (own_topictype, topictype)
                    except Exception as err:
                        import traceback
                        rclpy.logwarn(err)
                        rclpy.logwarn(traceback.format_exc())
        except:
            import traceback
            rclpy.logerr("SyncThread[%s] ERROR: %s", self.name, traceback.format_exc())
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
                rclpy.logdebug("    SyncThread[%s] clear all registrations", self.name)
                socket.setdefaulttimeout(5)
                own_master = xmlrpcclient.ServerProxy(self.masteruri_local)
                own_master_multi = xmlrpcclient.MultiCall(own_master)
                # end routine if the master was removed
                for topic, _topictype, node, uri in self.__subscriber:
                    rclpy.logdebug("    SyncThread[%s]   unsibscribe %s [%s]" % (self.name, topic, node))
                    own_master_multi.unregisterSubscriber(node, topic, uri)
                for topic, _topictype, node, uri in self.__publisher:
                    rclpy.logdebug("    SyncThread[%s]   unadvertise %s [%s]" % (self.name, topic, node))
                    own_master_multi.unregisterPublisher(node, topic, uri)
                for service, serviceuri, node, uri in self.__services:
                    rclpy.logdebug("    SyncThread[%s]   unregister service %s [%s]" % (self.name, service, node))
                    own_master_multi.unregisterService(node, service, serviceuri)
                rclpy.logdebug("    SyncThread[%s] execute a MultiCall", self.name)
                _ = own_master_multi()
                rclpy.logdebug("    SyncThread[%s] finished", self.name)
            except:
                rclpy.logerr("SyncThread[%s] ERROR while ending: %s", self.name, traceback.format_exc())
            socket.setdefaulttimeout(None)

    def _do_ignore_ntp(self, node, topic, topictype):
        if node == rclpy.get_name():
            return True
        return self._filter.is_ignored_publisher(node, topic, topictype)

    def _do_ignore_nts(self, node, topic, topictype):
        if node == rclpy.get_name():
            return True
        return self._filter.is_ignored_subscriber(node, topic, topictype)

    def _do_ignore_ns(self, node, service):
        if node == rclpy.get_name():
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
                # the node was registered originally to another ROS master -> do sync
                if masteruri != self.masteruri_local:
                    return uri
        return None

    def _get_serviceuri(self, service, nodes, remote_masteruri):
        for (servicename, uri, masteruri, _topic_type, local) in nodes:
            if (servicename == service) and ((self._filter.sync_remote_nodes() and masteruri == remote_masteruri) or local == 'local'):
                if masteruri != self.masteruri_local:
                    return uri
        return None