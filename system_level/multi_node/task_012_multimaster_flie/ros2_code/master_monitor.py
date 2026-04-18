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
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from fkie_master_discovery.master_info import MasterInfo
from fkie_master_discovery.common import masteruri_from_ros, get_hostname
from fkie_master_discovery.filter_interface import FilterInterface
from fkie_multimaster_msgs.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster, SyncMasterInfo, SyncTopicInfo, SyncServiceInfo
from fkie_multimaster_msgs.srv import DiscoverMasters, GetSyncInfo

class MasterConnectionException(Exception):
    '''
    The exception class to handle the connection problems with ROS Master.
    '''
    pass

class MasterMonitor(Node):
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
        super().__init__('master_monitor')
        self._state_access_lock = threading.RLock()
        self._create_access_lock = threading.RLock()
        self._lock = threading.RLock()
        self.__masteruri = masteruri_from_ros()
        self.__new_master_state = None
        self.__masteruri_rpc = None
        self.__mastername = None
        self.__cached_nodes = dict()
        self.__cached_services = dict()
        self.ros_node_name = self.get_name()
        if self.has_parameter('~name'):
            self.__mastername = self.get_parameter('~name').get_parameter_value().string_value
        self.__mastername = self.getMastername()
        self.set_parameter(Parameter('/mastername', Parameter.Type.STRING, self.__mastername))

        self.__master_state = None
        '''the current state of the ROS master'''
        self.rpcport = rpcport
        '''the port number of the RPC server'''

        self._printed_errors = dict()
        self._last_clearup_ts = time.time()

        self._master_errors = list()
        # Create an XML-RPC server
        self.ready = False
        while not self.ready and not rclpy.ok():
            try:
                # Create an XML-RPC server
                self.rpcServer = xmlrpc.server.SimpleXMLRPCServer((rpc_addr, rpcport), logRequests=False, allow_none=True)
                self.get_logger().info("Start RPC-XML Server at %s" % self.rpcServer.server_address)
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
                self.get_logger().warn("Error while start RPC-XML server on port %d: %s\nTry again..." % (rpcport, e))
                time.sleep(1)
            except:
                print(traceback.format_exc())
                if not do_retry:
                    raise

        self._master = xmlrpc.client.ServerProxy(self.getMasteruri())
        # Hide parameter
        self._re_hide_nodes = gen_pattern(self.get_parameter('~hide_nodes').get_parameter_value().string_value.split(','), 'hide_nodes')
        self._re_hide_topics = gen_pattern(self.get_parameter('~hide_topics').get_parameter_value().string_value.split(','), 'hide_topics')
        self._re_hide_services = gen_pattern(self.get_parameter('~hide_services').get_parameter_value().string_value.split(','), 'hide_services')
        # === UPDATE THE LAUNCH URIS Section ===
        # subscribe to get parameter updates
        self.get_logger().info("Subscribe to parameter `/roslaunch/uris`")
        self.__mycache_param_server = rclpy.parameter.ParameterServer(self)
        # HACK: use own method to get the updates also for parameters in the subgroup
        self.__mycache_param_server.update = self.__update_param
        # first access, make call to parameter server
        self._update_launch_uris_lock = threading.RLock()
        self.__launch_uris = {}
        code, msg, value = self._master.subscribeParam(self.ros_node_name, self.get_name(), '/roslaunch/uris')
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
                self.get_logger().info("Unsubscribe from parameter `/roslaunch/uris`")
                try:
                    self._master.unsubscribeParam(self.ros_node_name, self.get_name(), '/roslaunch/uris')
                except Exception as e:
                    self.get_logger().warn("Error while unsubscribe from `/roslaunch/uris`: %s" % e)
            self.get_logger().info("shutdown own RPC server")
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
                        launch_server = xmlrpc.client.ServerProxy(value)
                        c, m, pid = launch_server.get_pid()
                    except:
                        try:
                            # remove the parameter from parameter server on error
                            master = xmlrpc.client.ServerProxy(self.getMasteruri())
                            master.deleteParam(self.ros_node_name, key)
                        except:
                            pass
            finally:
                socket.setdefaulttimeout(None)
                # create the new timer
                if not rclpy.ok():
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
                    node = xmlrpc.client.ServerProxy(uri)
                    pid = self._succeed(node.getPid(self.ros_node_name))
                except (Exception, socket.error) as e:
                    with self._lock:
                        self._limited_log(nodename, "can't get PID: %s" % str(e), level=self.get_logger().debug)
                    master = xmlrpc.client.ServerProxy(self.getMasteruri())
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
                    dest_addr, dest_port = self.parse_rosrpc_uri(uri)
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
                    self.write_ros_handshake_header(s, header)
                    buf = io.StringIO() if sys.version_info < (3, 0) else io.BytesIO()
                    stype = self.read_ros_handshake_header(s, buf, 2048)
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
                        self._limited_log(service, "can't get service type: %s" % traceback.format_exc(), level=self.get_logger().debug)
                    with self._lock:
                        try:
                            del self.__cached_services[service]
                        except:
                            pass
                    pass
                finally:
                    if s is not None:
                        s.close()

    def _succeed(self, args):
        code, msg, val = args
        if code != 1:
            raise Exception("remote call failed: %s" % msg)
        return val

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
        '''
        Synchronize the local 'MasterInfo' object with the actual state of the ROS Master.
        
        :param clear_cache: The URI of nodes and services will be cached to reduce the load.
                            If remote hosted nodes or services was restarted, the cache must
                            be cleared! The local nodes will be updated periodically after
                            :mod:`fkie_master_discovery.master_monitor.MasterMonitor.MAX_PING_SEC`.
        
        :type clear_cache: bool (Default: ``False``)
        
        :return: ``True`` if the ROS master state is changed
        
        :rtype: bool
        '''
        self.__new_master_state = MasterInfo()
        try:
            # Establish communication with the ROS Master at the given 'masteruri' to retrieve 
            # the current system state (all publishers, subscribers, and services).
            code, msg, state = self._master.getSystemState(self.ros_node_name)
            if code == 1:
                # Extract and map the message types for all active topics.
                for topic, nodes in state[0].items():
                    for node in nodes:
                        self.__new_master_state.addPublisher(topic, node, self._get_nodeuri(node, state[2], self.getMasteruri()))
                for topic, nodes in state[1].items():
                    for node in nodes:
                        self.__new_master_state.addSubscriber(topic, node, self._get_nodeuri(node, state[2], self.getMasteruri()))
                for service, nodes in state[3].items():
                    for node in nodes:
                        self.__new_master_state.addService(service, self._get_serviceuri(service, state[4], self.getMasteruri()), node, self._get_nodeuri(node, state[2], self.getMasteruri()))
                # Update the internal state representation ('self.__new_master_state') such that it 
                # accurately reflects which nodes are associated with which topics and services.
                self.__new_master_state.timestamp = time.time()
                return True
            else:
                self.get_logger().error("Failed to retrieve system state from ROS Master: %s" % msg)
                return False
        except Exception as e:
            self.get_logger().error("Error updating Master state: %s" % str(e))
            return False

    def _limited_log(self, provider, msg, level):
        if provider not in self._printed_errors:
            self._printed_errors[provider] = dict()
        if msg not in self._printed_errors[provider]:
            self._printed_errors[provider][msg] = time.time()
            if level == self.get_logger().debug:
                self.get_logger().debug("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == self.get_logger().info:
                self.get_logger().info("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == self.get_logger().warn:
                self.get_logger().warn("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == self.get_logger().error:
                self.get_logger().error("MasterMonitor[%s]: %s" % (provider, msg))
            elif level == self.get_logger().fatal:
                self.get_logger().fatal("MasterMonitor[%s]: %s" % (provider, msg))

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
                        get_sync_info = self.create_client(GetSyncInfo, service.name)
                        try:
                            sync_info = get_sync_info.call_async(GetSyncInfo.Request())
                        except Exception as e:
                            self.get_logger().warn("ERROR Service call 'get_sync_info' failed: %s", str(e))
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
            master = xmlrpc.client.ServerProxy(self.__masteruri)
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
        self.get_logger().info('Set time: %s' % args)
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
                self.get_logger().warn(err)
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
                    self.get_logger().warn(timejump_msg)
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
        self.get_logger().warn('Shutdown yourself to avoid system instability because of time jump into past!\n')
        self.destroy_node()
        rclpy.try_shutdown()

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