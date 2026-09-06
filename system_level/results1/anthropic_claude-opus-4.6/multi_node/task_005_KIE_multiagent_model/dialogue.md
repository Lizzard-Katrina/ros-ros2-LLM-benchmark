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

FILE_PATH: interface_finder.py
----------------------------
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

import rospy
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
# TODO: [SYSTEM_LEVEL_MIGRATION_TASK]
# 1. Completely rewrite the '_get_topic' function to work with ROS 2.
# 2. Instead of using XML-RPC to query a ROS Master, use the ROS 2 Node's 
#    graph API (e.g., 'get_topic_names_and_types') to find topics of a specific type.
# 3. The function should still support the 'wait' and 'check_host' logic, 
#    ensuring it only returns topics matching the specified type and host requirements.
# END OF TODO

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
    while not result and not rospy.is_shutdown():
        master = xmlrpcclient.ServerProxy(masteruri)
        code, msg, val = master.getSystemState(rospy.get_name())
        if code == 1:
            pubs, subs, srvs = val
            own_host = get_hostname(masteruri)
            nodes_host = []
            # search for a service
            for srv, providers in srvs:
                if srv.endswith(name):
                    # only local service will be tacked
                    if check_host:
                        code, msg, val = master.lookupService(
                            rospy.get_name(), srv)
                        if code == 1:
                            hode_host = get_hostname(val)
                            if hode_host == own_host:
                                result.append(srv)
                            else:
                                nodes_host.append(hode_host)
                    else:
                        result.append(srv)
            if not result and wait:
                Log.warn(
                    f'mas-discovery node appear not to running @{own_host}, only found on {nodes_host}. Wait for service "{name}" @{own_host}.')
                time.sleep(1)
        elif not result and wait:
            Log.warn(f'cannot get state from ROS master: {code}, {msg}')
            time.sleep(1)
        if not wait:
            return result
    return result

----------------------------

FILE_PATH: master_discovery.py
----------------------------
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

try:
    import queue
except ImportError:
    import Queue as queue  # python 2 compatibility
import json
import errno
import rospy
import socket
import std_srvs.srv
import struct
import sys
import threading
import time
import traceback
try:
    import xmlrpclib as xmlrpcclient  # python 2 compatibility
except ImportError:
    import xmlrpc.client as xmlrpcclient

from rosgraph.network import get_local_addresses, get_local_address
from .common import get_hostname
from .master_monitor import MasterMonitor, MasterConnectionException
from .udp import DiscoverSocket, QueueReceiveItem, SEND_ERRORS
from fkie_mas_pylib.defines import MAX_ROS1_NETWORKS, NMD_DEFAULT_PORT
from fkie_mas_pylib.interface.runtime_interface import RosProvider
from fkie_mas_pylib.interface.runtime_interface import SystemWarning
from fkie_mas_pylib.interface.runtime_interface import SystemWarningGroup
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket import ws_port, ws_port_from


try:  # to avoid the problems with autodoc on ros.org/wiki site
    # , SyncMasterInfo, SyncTopicInfo
    from fkie_mas_msgs.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster
    from fkie_mas_msgs.srv import DiscoverMasters, DiscoverMastersResponse
except:
    pass


class DiscoveredMaster(object):
    '''
    The class stores all information about the remote ROS master and the all
    received heartbeat messages of the remote node. On first contact a threaded
    connection to remote discoverer will be established to get additional
    information about the ROS master.

    :param monitoruri: The URI of the remote RPC server, which monitor the ROS master

    :type monitoruri:  str

    :param heartbeat_rate: The remote rate, which is used to send the heartbeat messages.

    :type heartbeat_rate:  float (Default: `1.``)

    :param timestamp: The timestamp of the state of the remoter ROS master

    :type timestamp:  float (Default: ``0``)

    :param timestamp_local: The timestamp of the state of the remoter ROS master,
                            without the changes made during a synchronization.

    :type timestamp_local:  float (Default: ``0``)

    :param callback_master_state: the callback method to publish the changes of the ROS masters

    :type callback_master_state: `fkie_mas_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/MasterState.html>`_}  (Default: ``None``)
    '''

    MIN_HZ_FOR_QUALITY = 0.3

    ERR_RESOLVE_NAME = 1
    ERR_SOCKET = 2

    def __init__(self, monitoruri, is_local=False, heartbeat_rate=1.,
                 timestamp=0.0, timestamp_local=0.0, callback_master_state=None):
        '''
        Initialize method for the DiscoveredMaster class.

        :param monitoruri: The URI of the remote RPC server, which monitors the ROS master

        :type monitoruri:  str

        :param is_local: is the URI of the remote RPC server local or not

        :type is_local:  bool

        :param heartbeat_rate: The remote rate, which is used to send the heartbeat messages.

        :type heartbeat_rate:  float (Default: `1.``)

        :param timestamp: The timestamp of the state of the remoter ROS master

        :type timestamp:  float (Default: ``0``)

        :param timestamp_local: The timestamp of the state of the remoter ROS master, without the changes made during a synchronization.

        :type timestamp_local:  float (Default: ``0``)

        :param callback_master_state: the callback method to publish the changes of the ROS masters

        :type callback_master_state: `fkie_mas_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/MasterState.html>`_}  (Default: ``None``)
        '''
        self.__lock = threading.RLock()
        self.masteruri = None
        self.mastername = None
        self.timestamp = timestamp
        self.timestamp_local = timestamp_local
        self.discoverername = None
        self.monitoruri = monitoruri
        self.is_local = is_local
        self.heartbeat_rate = heartbeat_rate
        self.heartbeats = list()
        self.requests = list()
        self.last_heartbeat_ts = time.time()
        self.creation_ts = time.time()
        self.online = False
        self.callback_master_state = callback_master_state
        self.ts_last_request = 0
        self._errors = dict()  # ERR_*, msg
        self.monitor_hostname = get_hostname(monitoruri)
        self.master_hostname = None
        self.masteruriaddr = None
        self._on_finish = False
        # create a thread to retrieve additional information about the remote ROS master
        self._get_into_timer = threading.Timer(0.1, self._get_info_threaded)
        self._get_into_timer.start()

    def finish(self):
        self._on_finish = True
        try:
            self._get_into_timer.cancel()
        except Exception:
            pass

    def add_heartbeat(self, timestamp, timestamp_local, rate):
        '''
        Adds a new heartbeat measurement. If it is a new timestamp a ROS message
        about the change of this ROS master will be published into ROS network.

        :param timestamp: The new timestamp of the ROS master state

        :type timestamp:  float

        :param timestamp_local: The timestamp of the state of the remoter ROS
                          master, without the changes made during a synchronization.

        :type timestamp_local:  float (Default: ``0``)

        :param rate: The remote rate, which is used to send the heartbeat messages.
                     If the rate is zero the heartbeat is ignored.

        :type rate:  float

        :return: ``True`` on changes

        :rtype: bool
        '''
        result = False
        cur_time = time.time()
        self.last_heartbeat_ts = cur_time
        self.ts_last_request = 0
        self.requests = list()
        # publish new master state, if the timestamp is changed
        if (self.timestamp != timestamp or not self.online or self.timestamp_local != timestamp_local):
            self.timestamp = timestamp
            self.timestamp_local = timestamp_local
            if self.masteruri is not None:
                # set the state to 'online'
                self.online = True
                if self.callback_master_state is not None:
                    self.callback_master_state(MasterState(MasterState.STATE_CHANGED,
                                                           ROSMaster(str(self.mastername),
                                                                     self.masteruri,
                                                                     rospy.Time(
                                                                         self.timestamp),
                                                                     rospy.Time(
                                                                         self.timestamp_local),
                                                                     self.online,
                                                                     self.discoverername,
                                                                     self.monitoruri)))
                    result = True
        if rate >= DiscoveredMaster.MIN_HZ_FOR_QUALITY:
            # reset the list, if the heartbeat is changed
            if self.heartbeat_rate != rate:
                self.heartbeat_rate = rate
                self.heartbeats = list()
            self.heartbeats.append(cur_time)
        return result

    def add_request(self, timestamp):
        '''
        Stores the count of requests. This count is used to determine offline state
        of remote master.

        :param timestamp: time the request is sent.

        :type timestamp:  float
        '''
        self.ts_last_request = timestamp
        self.requests.append(timestamp)
        Log.debug("Unanswered requests [%d] for %s: %s" % (
            len(self.requests), str(self.masteruri), str(self.requests)))

    def requests_count(self):
        return len(self.requests)

    def remove_heartbeats(self, timestamp):
        '''
        Removes all heartbeat measurements, which are older as the given timestamp.

        :param timestamp: heartbeats older this timestamp will be removed.

        :type timestamp:  float

        :return: the count of removed heartbeats

        :rtype: int
        '''
        do_remove = True
        # remove the requests
        while do_remove:
            if len(self.requests) > 0 and self.requests[0] < timestamp:
                del self.requests[0]
            else:
                do_remove = False
        do_remove = True
        removed = 0
        while do_remove:
            if len(self.heartbeats) > 0 and self.heartbeats[0] < timestamp:
                del self.heartbeats[0]
                removed = removed + 1
            else:
                do_remove = False
        return removed

    def set_offline(self):
        '''
        Sets this master to offline and publish the new state to the ROS network.
        '''
        if self.online:
            self.online = False
            if self.callback_master_state is not None:
                Log.info('Set host to offline: %s' % self.mastername)
                self.callback_master_state(MasterState(MasterState.STATE_CHANGED,
                                                       ROSMaster(str(self.mastername),
                                                                 self.masteruri,
                                                                 rospy.Time(
                                                           self.timestamp),
                                                           rospy.Time(
                                                           self.timestamp_local),
                                                           False,
                                                           self.discoverername,
                                                           self.monitoruri)))

    def get_quality(self, interval=5, offline_after=1.4):
        '''
        Calculates the link quality to this master.
        '''
        quality = -1.0
        if self.mastername is not None and self.heartbeat_rate >= self.MIN_HZ_FOR_QUALITY:
            current_time = time.time()
            measurement_duration = interval
            if self.heartbeat_rate < 1.:
                measurement_duration = measurement_duration / self.heartbeat_rate
            # reduce the measurement duration on start of the master
            if measurement_duration > current_time - self.creation_ts:
                measurement_duration = current_time - self.creation_ts
            # remove all heartbeats, which are to old
            ts_oldest = current_time - measurement_duration
            self.remove_heartbeats(ts_oldest)
            # sets the master offline if the last received heartbeat is to old
            if current_time - self.last_heartbeat_ts > (measurement_duration * offline_after):
                self.set_offline()
            # calculate the quality for online masters only
            if self.online:
                beats_count = len(self.heartbeats)
                expected_count = int(
                    self.heartbeat_rate * measurement_duration + len(self.requests))
                if expected_count > 0:
                    quality = float(beats_count) / \
                        float(expected_count) * 100.0
                    if quality > 100.0:
                        quality = 100.0
        return quality

    @property
    def errors(self):
        '''
        Copies the errors and returns as dictionary.
        :return: a dictionary of error type (ERR_*) and a message
        '''
        result = dict()
        with self.__lock:
            for key, val in self._errors.items():
                result[key] = val
        return result

    def _add_error(self, error_id, msg):
        with self.__lock:
            if id not in self._errors:
                self._errors[error_id] = msg

    def _del_error(self, error_id):
        try:
            with self.__lock:
                del self._errors[error_id]
        except:
            pass

    def __start_get_info_timer(self, timetosleep):
        self._get_into_timer = threading.Timer(
            timetosleep, self._get_info_threaded)
        self._get_into_timer.start()

    def _get_info_threaded(self):
        thread = threading.Thread(target=self._retrieve_masterinfo)
        thread.setDaemon(True)
        thread.start()

    def _retrieve_masterinfo(self):
        '''
        Connects to the remote RPC server of the discoverer node and gets the
        information about the Master URI, name of the service, and other. The
        ``getMasterInfo()`` method will be used. On problems the connection will be
        reestablished until the information will be get successful.
        '''
        if self.monitoruri is not None and not self._on_finish:
            timetosleep = 5.
            if not rospy.is_shutdown() and self.mastername is None:
                try:
                    Log.debug(
                        "Get additional connection info from %s" % self.monitoruri)
                    remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
                    socket.setdefaulttimeout(10)
                    timestamp, masteruri, mastername, nodename, monitoruri = remote_monitor.masterContacts()
                    self._del_error(self.ERR_SOCKET)
                    Log.debug("Got [%s, %s, %s, %s] from %s" % (
                        timestamp, masteruri, mastername, nodename, monitoruri))
                    timetosleep = 0
                except socket.error as errobj:
                    msg = "can't retrieve connection information using XMLRPC from [%s], socket error: %s" % (
                        self.monitoruri, str(errobj))
                    Log.warn(msg)
                    self._add_error(self.ERR_SOCKET, msg)
                    if errobj.errno in [errno.EHOSTUNREACH]:
                        timetosleep = 30
                except:
                    msg = "can't retrieve connection information using XMLRPC from [%s]: %s" % (
                        self.monitoruri, traceback.format_exc())
                    Log.warn(msg)
                    self._add_error(self.ERR_SOCKET, msg)
                else:
                    if float(timestamp) != 0:
                        self.masteruri = masteruri
                        self.mastername = mastername
                        self.discoverername = nodename
#            self.monitoruri = monitoruri
                        self.timestamp = float(timestamp)
                        self.online = True
                        # resolve the masteruri. Print an error if not reachable
                        try:
                            self.master_hostname = get_hostname(self.masteruri)
                            self.masteruriaddr = socket.gethostbyname(
                                self.master_hostname)
                            self._del_error(self.ERR_RESOLVE_NAME)
                        except socket.gaierror:
                            msg = "Master discovered with not known hostname ROS_MASTER_URI:='%s'. Fix your network settings!" % str(
                                self.masteruri)
                            Log.warn(msg)
                            self._add_error(self.ERR_RESOLVE_NAME, msg)
                        except:
                            msg = "resolve error [%s]: %s" % (
                                self.monitoruri, traceback.format_exc())
                            Log.warn(msg)
                            self._add_error(self.ERR_SOCKET, msg)
                        else:
                            # publish new node
                            if self.callback_master_state is not None:
                                Log.info(
                                    "Added master with ROS_MASTER_URI=%s" % (self.masteruri))
                                self.callback_master_state(MasterState(MasterState.STATE_NEW,
                                                                       ROSMaster(str(self.mastername),
                                                                                 self.masteruri,
                                                                                 rospy.Time(
                                                                                     self.timestamp),
                                                                                 rospy.Time(
                                                                                     self.timestamp),
                                                                                 self.online,
                                                                                 self.discoverername,
                                                                                 self.monitoruri)))
                                timetosleep = 0
                            else:
                                msg = "callback is None, should not happen...remove master %s" % self.monitoruri
                                Log.warn(msg)
                                self._add_error(self.ERR_SOCKET, msg)
                    else:
                        msg = "Got timestamp=0 from %s, retry... " % self.monitoruri
                        Log.warn(msg)
                        self._add_error(self.ERR_SOCKET, msg)
                finally:
                    socket.setdefaulttimeout(None)
                if not self._on_finish and timetosleep > 0:
                    self.__start_get_info_timer(timetosleep)


class Discoverer(object):
    '''
    The class to publish the current state of the ROS master.

    Discovering is done by heartbeats:
      Each master discovery node sends to a multicast group periodically messages
      with current state. If the frequency is less than 0.3 the detected changes
      on ROS master are published immediately.
      The current state is described by timestamp of last change. The frequency of
      heartbeats can be changed by `~heartbeat_hz` parameter.

      If heartbeats are disabled (`~heartbeat_hz` is zero) each master discovery
      node sends on start three notification messages and requests.

      If for a host no more heartbeat are received while `ACTIVE_REQUEST_AFTER (60 sec)`
      a request to this host will be sent as an unicast message. After five
      unanswered requests the host state will be changed to `offline`.
      After `REMOVE_AFTER (300 sec)` the host will be removed.

    :param mcast_port: The port used to publish and receive the multicast messages.

    :type mcast_port:  int

    :param mcast_group: The IPv4 or IPv6 multicast group used for discovering over nodes.

    :type mcast_group:  str

    :param monitor_port: The port of the RPC Server, used to get more information about the ROS master.

    :type monitor_port:  int
    '''

    VERSION = 2
    '''the version of the packet format described by ``HEARTBEAT_FMT``

      :Version 1: 'cBBiiH'

      ::

        one character 'R'
        unsigned char: version of the heartbeat message
        unsigned char: rate of the heartbeat message in HZ*10. Maximal rate: 25.5 Hz -> value 255
        int: secs of the ROS Master state
        int: nsecs of the ROS Master state
        unsigned short: the port number of the RPC Server of the remote ROS-Core monitor

      :Version 2: 'cBBiiHii'

      ::

        ``Version 1``
        int: secs of the ROS Master state (only local changes). Changes while sync will be ignored.
        int: nsecs of the ROS Master state (only local changes). Changes while sync will be ignored.

      :Version 3: 'cBBiiHii'

      ::

        ``Version 2``
        if the timestamp of ROS Master state is zero, the reply as unicast
        message will be send to the sender.

  '''
    HEARTBEAT_FMT = 'cBBiiHii'
    ''' packet format description, see: http://docs.python.org/library/struct.html '''
    HEARTBEAT_HZ = 0.02
    ''' the send rate of the heartbeat packets in hz. Zero disables the heartbeats. (Default: 0.02 Hz)
      Only values between 0.1 and 25.5 are used to determine the link quality.
  '''
    MEASUREMENT_INTERVALS = 5
    ''' the count of intervals (1 sec) used for a quality calculation. If
      `HEARTBEAT_HZ` is smaller then 1, `MEASUREMENT_INTERVALS` will be divided
      by `HEARTBEAT_HZ` value.
      (Default: 5 sec are used to determine the link quality)'''
    TIMEOUT_FACTOR = 1
    ''' the timeout is defined by calculated measurement duration multiplied by `TIMEOUT_FACTOR`. '''
    ROSMASTER_HZ = 1
    ''' the test rate of ROS master state in Hz (Default: 1 Hz). '''
    REMOVE_AFTER = 300
    ''' remove an offline host after this time in [sec] (Default: 300 sec). '''

    ACTIVE_REQUEST_AFTER = 60
    ''' send an update request, if after this time no heartbeats are received [sec] (Default: 60 sec). '''

    INIT_NOTIFICATION_COUNT = 3
    ''' the count of heartbeats and update request to send at the start (Default: 3 sec).
      It will be send with 1Hz. Only used if `HEARTBEAT_HZ` is zero. '''

    OFFLINE_AFTER_REQUEST_COUNT = 5
    ''' After this unanswered count of requests for update the remote master is set
      to offline state (Default: 5 sec).
      The requests are send after `ACTIVE_REQUEST_AFTER` with `ROSMASTER_HZ`. '''

    CHANGE_NOTIFICATION_COUNT = 3
    ''' After the ROS master was changed the new state will be sent for
      `CHANGE_NOTIFICATION_COUNT` times (Default: 3 sec). The new state will be
      sent with `ROSMASTER_HZ` and only if `HEARTBEAT_HZ` is zero. '''

    NETPACKET_SIZE = 68

    def __init__(self, mcast_port, mcast_group, monitor_port, rpc_addr='', ws_port=ws_port()):
        '''
        Initialize method for the Discoverer class

        :param mcast_port: The port used to publish and receive the multicast messages.

        :type mcast_port:  int

        :param mcast_group: The IPv4 or IPv6 multicast group used for discovering over nodes.

        :type mcast_group:  str

        :param monitor_port: The port of the RPC Server, used to get more information about the ROS master.

        :type monitor_port:  int
        '''
#    threading.Thread.__init__(self)
        self.do_finish = False
        self._services_initialized = False
        self.__lock = threading.RLock()
        # the list with all ROS master neighbors
        self.masters = dict()  # (ip, DiscoveredMaster)
        # this parameter stores the state of the remote nodes. If the state is changed
        # the cache for contacts of remote nodes will be cleared.
        self._changed = False
        self._json_reported_masters = set()
        self._last_datetime = time.time()

        # TODO
        # initialize ROS 2 publishers for '~/linkstats' (LinkStatesStamped) 
#    and '~/changes' (MasterState) using the node's create_publisher method.
        # END OF TODO

        # for cases with more then one master_discovery on the same host and
        # heartbeat rate is less then 0.1. In this case we have to send a multicast
        # request reply, because we are bind to the same port. Unicast replies are
        # not forward to the same port only once.
        self._addresses = dict()  # {address : (int) ocurres}
        # some parameter checks and info outputs
        if not self._send_mcast:
            Log.warn("Send multicast is disabled.")
        if not self._listen_mcast:
            Log.warn("Listen to multicast is disabled.")
        if not self._send_mcast and not self.robots:
            Log.warn(
                "This master_discovery is invisible because it sends no heart beat messages! Set ~send_mcast/~listen_mcast to True or add hosts to ~robot_hosts.")
        if not self._listen_mcast or not self._send_mcast:
            self.robots.append('localhost')
        self.robots = list(set(self.robots))
        Log.info("Check the ROS Master[Hz]: " + str(self.ROSMASTER_HZ))
        if self.HEARTBEAT_HZ <= 0.:
            Log.warn(
                "Heart beat [Hz]: %s is increased to 0.02" % self.HEARTBEAT_HZ)
            self.HEARTBEAT_HZ = 0.02
        if self.HEARTBEAT_HZ > 25.5:
            Log.warn(
                "Heart beat [Hz]: %s is decreased to 25.5" % self.HEARTBEAT_HZ)
            self.HEARTBEAT_HZ = 25.5
        else:
            Log.info("Heart beat [Hz]: %s" % (self.HEARTBEAT_HZ))
        Log.info(
            "Active request after [sec]: %s" % self.ACTIVE_REQUEST_AFTER)
        Log.info("Remove after [sec]: %s" % self.REMOVE_AFTER)
        if self.REMOVE_AFTER <= self.ACTIVE_REQUEST_AFTER:
            Log.warn(
                "'Active request after' should be less than 'remove after' to avoid removing masters from list!")
        Log.info("Robot hosts: " + str(self.robots))
        if self.HEARTBEAT_HZ > 0.:
            count_packets = len(self.robots) + (1 if self._send_mcast else 0)
            netload = self.HEARTBEAT_HZ * self.NETPACKET_SIZE * count_packets
            Log.info(
                "Approx. mininum avg. network load: %.2f bytes/s" % netload)
        self.current_check_hz = self.ROSMASTER_HZ
        self.pubstats = rospy.Publisher(
            "~linkstats", LinkStatesStamped, queue_size=1)
        # test the reachability of the ROS master
        local_addr = get_local_address()
        if (local_addr in ['localhost', '127.0.0.1']):
            Log.warn(
                "'%s' is not reachable for other systems. Change the ROS_MASTER_URI!" % local_addr)
        self.mcast_port = mcast_port
        self.mcast_group = mcast_group
        self._ts_received_mcast_request = 0
        # initialize the ROS publishers
        self.pubchanges = rospy.Publisher(
            "~changes", MasterState, queue_size=10)
        # create a thread to monitor the ROS master state
        mgroup = DiscoverSocket.normalize_mgroup(mcast_group)
        is_ip6 = self._is_ipv6_group(mgroup)
        self.master_monitor = MasterMonitor(
            monitor_port, ipv6=is_ip6, rpc_addr=rpc_addr, connect_server=True, ws_port=ws_port)
        # create timer to check for ros master changes
        self._timer_ros_changes = threading.Timer(
            0.1, self.checkROSMaster_loop)
        # init socket for discovering. Exit on errors!
        self._init_socket(True)
        # create a timer monitor the offline ROS master and calculate the link qualities
        self._timer_stats = threading.Timer(1, self.timed_stats_calculation)
        # create timer and parameter for heartbeat notifications
        self._init_notifications = 0
        # disable parameter, if HEARTBEAT_HZ is active (> zero)
        if self.HEARTBEAT_HZ > DiscoveredMaster.MIN_HZ_FOR_QUALITY:
            # send init requests in mixed scenario: self._init_notifications = self.INIT_NOTIFICATION_COUNT
            self._current_change_notification_count = self.CHANGE_NOTIFICATION_COUNT
        self._timer_heartbeat = threading.Timer(1.0, self.send_heartbeat)
        # set the callback to finish all running threads
        rospy.on_shutdown(self.on_shutdown)
        self._recv_tread = threading.Thread(target=self._recv_loop_from_queue)

    def start(self):
        self._recv_tread.start()
        self._timer_ros_changes.start()
        self._timer_stats.start()
        self._timer_heartbeat.start()

    def _is_ipv6_group(self, addr):
        try:
            socket.inet_pton(socket.AF_INET6, addr)
            return True
        except:
            pass
        return False

    def _init_socket(self, doexit_on_error=False):
        # create discovery socket
        # if multicast messages are disabled only unicast socket is created
        # unicast socket is also created if ~interface is defined
        self.socket = DiscoverSocket(self.mcast_port, self.mcast_group,
                                     send_mcast=self._send_mcast, listen_mcast=self._listen_mcast)
        if self._send_mcast or self._listen_mcast:
            if not self.socket.hasEnabledMulticastIface() and doexit_on_error:
                sys.exit("No enabled multicast interfaces available!\nAdd multicast support e.g. sudo ifconfig eth0 multicast or disable multicast by settings 'send_mcast' and 'listen_mcast' to False.\nExit")
        # set callback for received UDP messages
#        self.socket.set_message_callback(self.recv_udp_msg)

    def _stop_timers(self):
        try:
            self._timer_ros_changes.cancel()
        except:
            pass
        try:
            self._timer_heartbeat.cancel()
        except:
            pass
        try:
            self._timer_stats.cancel()
        except:
            pass

    def on_shutdown(self, *arg):
        with self.__lock:
            # tell other loops to finish
            self.do_finish = True
            for (_, master) in self.masters.items():
                if master.mastername is not None:
                    self.publish_masterstate(MasterState(MasterState.STATE_REMOVED,
                                                         ROSMaster(str(master.mastername),
                                                                   master.masteruri,
                                                                   rospy.Time(
                                                                       master.timestamp),
                                                                   rospy.Time(
                                                                       master.timestamp_local),
                                                                   master.online,
                                                                   master.discoverername,
                                                                   master.monitoruri)))
                master.finish()
            # send notification that the master is going off
            msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                              int(self.HEARTBEAT_HZ * 10), -1, -1,
                              self.master_monitor.rpc_port, -1, -1)
            self._publish_current_state(msg=msg)
            self.masters.clear()
            self._json_publish_masters()
            # finish the RPC server and timer
            self.master_monitor.shutdown()
            time.sleep(0.2)

    def finish(self):
        '''
        Callback called on exit of the ros node and publish the empty list of
        ROSMasters.
        '''
        # publish all master as removed
        Log.debug("Finish master discovery")
        self._stop_timers()
        self.socket.close()
        self._killme_timer = threading.Timer(19., self._killme)
        self._killme_timer.setDaemon(True)
        self._killme_timer.start()

    def _killme(self):
        import os
        import signal
        os.kill(os.getpid(), signal.SIGKILL)

    def send_heartbeat(self, timer=True):
        '''
        Sends current state as heartbeat messages to defined multicast group. If the
        Discoverer.HEARTBEAT_HZ is greather then zero a timer will be started to
        send heartbeat messages periodically. This message will also send on start
        of the discoverer.
        '''
        with self.__lock:
            # publish the current state
            invalid_uri = (self.master_monitor.getMasteruri() is None)
            invalid_state = (self.master_monitor.getCurrentState() is None)
            if not (invalid_uri or invalid_state or rospy.is_shutdown() or self.do_finish):
                self._publish_current_state()
                # send update requests to group
                if timer and self._listen_mcast and self._init_notifications < self.INIT_NOTIFICATION_COUNT:
                    self._init_notifications += 1
                    Log.debug("Send requests while init %d/%d" %
                              (self._init_notifications, self.INIT_NOTIFICATION_COUNT))
                    self._request_state()
            if timer and not self.do_finish:
                if (self.HEARTBEAT_HZ > 0. or self._init_notifications < self.INIT_NOTIFICATION_COUNT):
                    sleeptime = 1.0 / self.HEARTBEAT_HZ if self.HEARTBEAT_HZ > 0. else 1.0
                    Log.debug(
                        "Set timer to send heartbeat in %.2f sec" % sleeptime)
                    self._timer_heartbeat = threading.Timer(
                        sleeptime, self.send_heartbeat)
                    self._timer_heartbeat.start()

    def _publish_current_state(self, address=None, msg=None):
       # TODO
#    a. Handle logic for sending heartbeat messages via the 'self.socket'.
#    b. If 'msg' is None, generate a new heartbeat packet using 'struct.pack' 
#       with the current ROS 2 timestamp (secs/nsecs) and RPC port.
#    c. Support sending to a specific 'address' (unicast) or to the multicast group.
     # END OF TODO
    def _request_state(self, addresses=None, masters=[]):
        try:
            msg = self._create_request_update_msg()
            current_time = time.time()
            for master in masters:
                master.add_request(current_time)
            if addresses is not None:
                Log.debug(
                    'Send a unicast request for update to %s' % addresses)
                self.socket.send_queued(msg, addresses)
            elif self._send_mcast:
                if current_time - self._ts_received_mcast_request > 1. / self.current_check_hz:
                    Log.debug('Send request to mcast group %s:%s' %
                              (self.mcast_group, self.mcast_port))
                    # do not send a multicast request if one was received in last time
                    self.socket.send_queued(msg)
                else:
                    Log.debug('Skipped send request to mcast group %s:%s. Last send was %.2fsec ago,  allowed %.2f' % (
                        self.mcast_group, self.mcast_port, current_time - self._ts_received_mcast_request, 1. / self.current_check_hz))
            if self.robots and addresses is None:
                Log.debug(
                    'Send request as unicast to all robot hosts %s' % self.robots)
                self.socket.send_queued(msg, self.robots)
        except Exception as e:
            print(traceback.format_exc())
            Log.warn("Send with addresses '%s' failed: %s" %
                     (addresses, e))

    def _create_current_state_msg(self):
        t = 0
        local_t = 0
        if not self.master_monitor.getCurrentState() is None:
            t = self.master_monitor.getCurrentState().timestamp
            local_t = self.master_monitor.getCurrentState().timestamp_local
            return struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                               int(self.HEARTBEAT_HZ * 10),
                               int(t), int((t - (int(t))) * 1000000000),
                               self.master_monitor.rpc_port,
                               int(local_t), int((local_t - (int(local_t))) * 1000000000))
        return None

    def _create_request_update_msg(self):
        version = Discoverer.VERSION if Discoverer.VERSION > 2 else 3
        msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', version,
                          int(self.HEARTBEAT_HZ * 10), 0, 0,
                          self.master_monitor.rpc_port, 0, 0)
        return msg

    def checkROSMaster_loop(self):
        '''
        The method test periodically the state of the ROS master. The new state will
        be published as heartbeat messages.
        :mod:`fkie_mas_discovery.master_monitor.MasterMonitor.checkState()`
        '''
        import os
        try_count = 0
        if (not rospy.is_shutdown()) and not self.do_finish:
            try:
                cputimes = os.times()
                cputime_init = cputimes[0] + cputimes[1]
                self.update_master_errors()
                if self.master_monitor.checkState(self._changed):
                    # publish the new state if frequetly publishing is disabled
                    if not self.do_finish and self.HEARTBEAT_HZ < DiscoveredMaster.MIN_HZ_FOR_QUALITY:
                        self.send_heartbeat(timer=False)
                        self._current_change_notification_count = 0
                with self.__lock:
                    self._changed = False
                # repeat the last change for `CHANGE_NOTIFICATION_COUNT` times
                if 0 < self._current_change_notification_count < self.CHANGE_NOTIFICATION_COUNT:
                    self._current_change_notification_count += 1
                    self.send_heartbeat(timer=False)
                # adapt the check rate to the CPU usage time
                cputimes = os.times()
                cputime = cputimes[0] + cputimes[1] - cputime_init
                if self.current_check_hz * cputime > 0.20:
                    self.current_check_hz = float(self.current_check_hz) / 2.0
                elif self.current_check_hz * cputime < 0.10 and float(self.current_check_hz) * 2.0 < self.ROSMASTER_HZ:
                    self.current_check_hz = float(self.current_check_hz) * 2.0
                try_count = 0
            except MasterConnectionException as conn_err:
                try_count = try_count + 1
                if try_count == 5:
                    Log.error("Communication with ROS Master failed:", conn_err)

            # remove offline hosts or request updates
            self._remove_offline_hosts()
            # setup timer for next ROS master state check
            if not rospy.is_shutdown():
                self._timer_ros_changes = threading.Timer(
                    1.0 / self.current_check_hz, self.checkROSMaster_loop)
                self._timer_ros_changes.start()

    def _remove_offline_hosts(self):
        with self.__lock:
            current_time = time.time()
            to_remove = []
            multi_address = []
            for (k, v) in self.masters.items():
                ts_since_last_hb = current_time - v.last_heartbeat_ts
                ts_since_last_request = current_time - \
                    max(v.ts_last_request, v.last_heartbeat_ts)
                if self.REMOVE_AFTER > 0 and ts_since_last_hb > self.REMOVE_AFTER:
                    to_remove.append(k)
                    if v.mastername is not None:
                        self.publish_masterstate(MasterState(MasterState.STATE_REMOVED,
                                                             ROSMaster(str(v.mastername),
                                                                       v.masteruri,
                                                                       rospy.Time(
                                                                           v.timestamp),
                                                                       rospy.Time(
                                                                           v.timestamp_local),
                                                                       v.online,
                                                                       v.discoverername,
                                                                       v.monitoruri)))
                        v.finish()
                # request updates
                elif ts_since_last_request > self.ACTIVE_REQUEST_AFTER or (v.requests_count() > 0 and v.online):
                    if v.requests_count() >= self.OFFLINE_AFTER_REQUEST_COUNT:
                        v.set_offline()
                    if self._is_multi_address(k[0][0]):
                        # in case more then one master_discovery runs on the same address
                        # we send also a multicast message, because no two masters can bind to
                        # one unicast address
                        multi_address.append(v)
                    else:
                        self._request_state(k[0][0], [v])
            if multi_address:
                self._request_state(masters=multi_address)
            for r in to_remove:
                Log.info(
                    "Remove master discovery: http://%s:%s" % (r[0][0], r[1]))
                self._rem_address(r[0][0])
                del self.masters[r]
            if to_remove:
                self._json_publish_masters()

    def _recv_loop_from_queue(self):
        while not self.do_finish:
            try:
                recv_item = self.socket.receive_queue.get(timeout=1)
                self.recv_udp_msg(
                    recv_item.msg, recv_item.sender_addr, recv_item.via)
            except queue.Empty:
                pass

    def recv_udp_msg(self, msg, address, via):
        '''
        This method handles the received udp messages.
        '''
        if not rospy.is_shutdown() and not self.do_finish:
            with self.__lock:
                self._check_timejump()
                try:
                    if len(msg) == 0:
                        return
                    (version, msg_tuple) = self.msg2masterState(msg, address)
                    if (version in [2, 3]):
                        add_to_list = False
                        (firstc, version, rate, secs, nsecs,
                         monitor_port, secs_l, nsecs_l) = msg_tuple
                        if firstc != b'R':
                            # ignore the message. it does not start with 'R'
                            return
                        master_key = (address, monitor_port)
                        if version >= 3 and secs == 0 and nsecs == 0:
                            # is it a request to update the state
                            # send the current master state to the sender address
                            # TODO: add a filter, if multicast messages are disabled?
                            if self.master_monitor.getCurrentState() is not None:
                                if via == QueueReceiveItem.MULTICAST:
                                    Log.debug(
                                        "Received a multicast request for a state update from %s" % address[0])
                                    self._ts_received_mcast_request = time.time()
                                    if self._send_mcast:
                                        self._publish_current_state()
                                    self._publish_current_state(address[0])
                                elif via in [QueueReceiveItem.LOOPBACK, QueueReceiveItem.UNICAST]:
                                    Log.debug(
                                        "Received a request for a state update from %s" % (address[0]))
                                    self._publish_current_state(address[0])
                            add_to_list = master_key not in self.masters
                        elif secs == -1 or secs_l == -1:
                            # remove master if sec and nsec are -1
                            Log.debug(
                                "Received a LEAVE heartbeat from %s via %s socket" % (master_key[0], via))
                            if master_key in self.masters:
                                master = self.masters[master_key]
                                if master.mastername is not None:
                                    # the contact info of the master is valied, publish the change
                                    state_remove = MasterState(MasterState.STATE_REMOVED,
                                                               ROSMaster(str(master.mastername),
                                                                         master.masteruri,
                                                                         rospy.Time(
                                                                             master.timestamp),
                                                                         rospy.Time(
                                                                             master.timestamp_local),
                                                                         False,
                                                                         master.discoverername,
                                                                         master.monitoruri))
                                    master.finish()
                                    self.publish_masterstate(state_remove)
                                Log.info("Remove master discovery: http://%s:%s, with ROS_MASTER_URI=%s" % (
                                    address[0], monitor_port, master.masteruri))
                                self._rem_address(address[0])
                                del self.masters[master_key]
                                self._json_publish_masters()
                        elif master_key in self.masters:
                            # update the timestamp of existing master
                            Log.debug(
                                "Received a heartbeat from %s via %s socket" % (master_key[0], via))
                            changed = self.masters[master_key].add_heartbeat(float(
                                secs) + float(nsecs) / 1000000000.0, float(secs_l) + float(nsecs_l) / 1000000000.0, float(rate) / 10.0,)
                            if not self._changed:
                                self._changed = changed
                        else:
                            Log.debug(
                                "Received a NEW heartbeat from %s via %s socket" % (master_key[0], via))
                            # or create a new master
                            add_to_list = True
                        if add_to_list:
                            Log.info(
                                "Detected master discovery: http://%s:%s" % (address[0], monitor_port))
                            self._add_address(address[0])
                            is_local = address[0].startswith(
                                '127.') or address[0] in get_local_addresses()
                            self.masters[master_key] = DiscoveredMaster(monitoruri=''.join(['http://', address[0], ':', str(monitor_port)]),
                                                                        is_local=is_local,
                                                                        heartbeat_rate=float(
                                                                            rate) / 10.0,
                                                                        timestamp=float(
                                                                            secs) + float(nsecs) / 1000000000.0,
                                                                        timestamp_local=float(
                                                                            secs_l) + float(nsecs_l) / 1000000000.0,
                                                                        callback_master_state=self.publish_masterstate)
                            if via == QueueReceiveItem.LOOPBACK:
                                self._publish_current_state(address[0])
                except Exception as e:
                    Log.warn("Error while decode message: %s", str(e))

    def _json_publish_masters(self):
        try:
            json_reported_masters = set()
            result = []
            for (addr, port), master in self.masters.items():
                # check for master.online
                cbmaster = RosProvider(name=master.mastername if master.mastername and len(master.mastername) > 0 else f'{addr}:{port}',
                                       host=addr[0],
                                       port=ws_port_from(self.mcast_port, master.masteruri),
                                       masteruri=master.masteruri if len(
                    master.masteruri) > 0 else f'{addr}:{port}',
                    origin=master.masteruri == self.master_monitor.getMasteruri(),
                    hostnames=list(set([addr[0], get_hostname(master.masteruri)])))
                cbmaster.ros_domain_id = self.mcast_port - 11511
                result.append(cbmaster)
                json_reported_masters.add(master.masteruri)
            if not (json_reported_masters == self._json_reported_masters):
                self.master_monitor.setProviderList(result)
                self._json_reported_masters = json_reported_masters
        except Exception as cpe:
            import traceback
            Log.warn(traceback.format_exc())

    def _check_timejump(self):
        if self._last_datetime > time.time():
            self._stop_timers()
            self.checkROSMaster_loop()
            self.send_heartbeat(True)
            self.timed_stats_calculation()
        self._last_datetime = time.time()

    def _is_multi_address(self, address):
        return address in self._addresses and self._addresses[address] > 1

    def _add_address(self, address):
        if address in self._addresses:
            self._addresses[address] += 1
        else:
            self._addresses[address] = 1

    def _rem_address(self, address):
        if address in self._addresses:
            self._addresses[address] -= 1
        if self._addresses[address] == 0:
            del self._addresses[address]

    @classmethod
    def msg2masterState(cls, msg, address):
        '''
        :return: parses the heartbeat message and return a tuple of
                version and values corresponding with current version of message.
                :mod:`fkie_mas_discovery.master_discovery.Discoverer.HEARTBEAT_FMT`

        :raise: Exception on invalid message

        :rtype: (``unsigned char``, tuple corresponding to :mod:`fkie_mas_discovery.master_discovery.Discoverer.HEARTBEAT_FMT`)
        '''
        if len(msg) > 2:
            (r,) = struct.unpack('c', msg[0:1])
            (version,) = struct.unpack('B', msg[1:2])
            if (version in [Discoverer.VERSION, 2, 3]):
                if (r == b'R'):
                    struct_size = struct.calcsize(Discoverer.HEARTBEAT_FMT)
                    if len(msg) == struct_size:
                        return (version, struct.unpack(Discoverer.HEARTBEAT_FMT, msg))
                    else:
                        raise Exception("wrong message size; expected %d, got %d from %s" % (
                            struct_size, len(msg), address))
                else:
                    raise Exception(
                        "wrong initial discovery message char %s received from %s" % (r, address))
            elif (version > Discoverer.VERSION):
                raise Exception("newer heartbeat version %s (own: %s) from %s detected, please update your master_discovery" % (
                    version, Discoverer.VERSION, address))
            elif (version < Discoverer.VERSION):
                raise Exception("old heartbeat version %s detected (current: %s), please update master_discovery on %s" % (
                    version, Discoverer.VERSION, address))
            else:
                raise Exception("heartbeat version %s expected, received: %s" % (
                    Discoverer.VERSION, version))
        raise Exception("message is too small")

    def timed_stats_calculation(self):
        '''
        This method will be called by a timer and has two jobs:
         1. set the masters offline, if no heartbeat messages are received a long time
         2. calculate the quality of known links
        '''
        result = LinkStatesStamped()
        result.header.stamp = rospy.Time.from_sec(time.time())
        with self.__lock:
            for (_, v) in self.masters.items():
                quality = v.get_quality(
                    self.MEASUREMENT_INTERVALS, self.TIMEOUT_FACTOR)
                if not (v.mastername is None) and v.online:
                    result.links.append(
                        LinkState(v.mastername, quality, rospy.Time.from_sec(v.last_heartbeat_ts)))
                if v.is_local:
                    result.header.frame_id = v.mastername
        # publish the results
        self.publish_stats(result)
        try:
            if not rospy.is_shutdown():
                self._timer_stats = threading.Timer(
                    1, self.timed_stats_calculation)
                self._timer_stats.start()
        except:
            pass

    def publish_masterstate(self, master_state):
        '''
        Publishes the given state to the ROS network. This method is thread safe.

        :param master_state: the master state to publish

        :type master_state:  `fkie_mas_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/MasterState.html>`_
        '''
        with self.__lock:
            try:
                self.pubchanges.publish(master_state)
                if not self._services_initialized:
                    # initialize the ROS services
                    self._services_initialized = True
                    rospy.Service('~list_masters', DiscoverMasters,
                                  self.rosservice_list_masters)
                    rospy.Service('~refresh', std_srvs.srv.Empty,
                                  self.rosservice_refresh)
                if master_state.state in [MasterState.STATE_NEW, MasterState.STATE_CHANGED]:
                    self._json_publish_masters()
            except:
                traceback.print_exc()

    def publish_stats(self, stats):
        '''
        Publishes the link quality states to the ROS network.This method is thread safe.

        :param stats: the link quality states to publish

        :type stats:  `fkie_mas_discovery.msg.LinkStatesStamped <http://www.ros.org/doc/api/fkie_mas_discovery/html/msg/LinkStatesStamped.html>`_
        '''
        if not rospy.is_shutdown():
            with self.__lock:
                try:
                    self.pubstats.publish(stats)
                except:
                    traceback.print_exc()

    def update_master_errors(self):
        result = []
        json_w_ip_mismatch = SystemWarningGroup(
            SystemWarningGroup.ID_ADDR_MISMATCH)
        json_w_resolve = SystemWarningGroup(
            SystemWarningGroup.ID_RESOLVE_FAILED)
        json_w_udp_send = SystemWarningGroup(
            SystemWarningGroup.ID_UDP_SEND)
        json_w_exception = SystemWarningGroup(
            SystemWarningGroup.ID_EXCEPTION)
        json_warnings = [json_w_ip_mismatch, json_w_resolve,
                         json_w_udp_send, json_w_exception]
        with self.__lock:
            try:
                current_errors = self.master_monitor.getMasterErrors()[1]
                for (_, v) in self.masters.items():
                    # add all errors to the responce
                    for _, msg in v.errors.items():
                        result.append(msg)
                    # test for resolved addr
                    if v.mastername is not None and not v.errors and v.masteruri != self.master_monitor.getMasteruri():
                        try:
                            if v.masteruriaddr != v.monitor_hostname:
                                msg_err = f"Resolved host of ROS_MASTER_URI {v.master_hostname}={v.masteruriaddr} and origin discovered IP={v.monitor_hostname} are different"
                                msg_hint = f"Fix your network settings (e.g. /etc/hosts) and restart master_discovery!"
                                msg = f"{msg_err} {msg_hint}"
                                if v.masteruriaddr is None or not v.masteruriaddr.startswith('127.'):
                                    local_addresses = [
                                        'localhost'] + get_local_addresses()
                                    # check 127/8 and local addresses
                                    if v.masteruriaddr not in local_addresses:
                                        if msg not in current_errors:
                                            Log.warn(msg)
                                        result.append(msg)
                                        json_w_ip_mismatch.append(
                                            SystemWarning(msg=msg_err, hint=msg_hint))
                        except Exception as e:
                            msg_err = f"Error while resolve address for {v.masteruri}: {str(e)}"
                            result.append(msg_err)
                            json_w_resolve.append(
                                SystemWarning(msg=msg_err))
                            Log.warn(msg_err)
                try:
                    for _addr, msg in SEND_ERRORS.items():
                        result.append('%s' % msg)
                        json_w_udp_send.append(SystemWarning(msg=msg))
                except:
                    pass
            except Exception as e:
                msg_err = f"{str(e)}"
                result.append(msg_err)
                Log.warn(msg_err)
                json_w_exception.append(SystemWarning(msg=msg_err))
        self.master_monitor.update_master_errors(result)
        self.master_monitor.update_errors_json(json_warnings)

    def rosservice_list_masters(self, req):
        '''
        Callback for the ROS service to get the current list of the known ROS masters.
        '''
        masters = list()
        with self.__lock:
            try:
                for (_, v) in self.masters.items():
                    if v.mastername is not None:
                        masters.append(ROSMaster(str(v.mastername),
                                                 v.masteruri,
                                                 rospy.Time(v.timestamp),
                                                 rospy.Time(v.timestamp_local),
                                                 v.online,
                                                 v.discoverername,
                                                 v.monitoruri))
            except:
                traceback.print_exc()
        return DiscoverMastersResponse(masters)

    def rosservice_refresh(self, req):
        '''
        Callback for the ROS service to send an active unicast and multicast request
        to each known master discovery.
        '''
        with self.__lock:
            try:
                for (k, v) in self.masters.items():
                    if v.mastername is not None:
                        # send an active unicast request
                        self._request_state(k[0][0], [v])
                if self._listen_mcast:
                    self._request_state()
#        self._send_current_state()
            except:
                traceback.print_exc()
        return []

----------------------------


# LLM Response



[FILENAME: interface_finder.py]
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
from .common import get_hostname
from fkie_mas_pylib.logging.logging import Log


def get_changes_topic(node, wait=True, check_host=True):
    '''
    Search in publishers of ROS 2 for a topic with type containing `MasterState` and
    returns his name. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param node: the ROS 2 node used to query the graph

    :type node: rclpy.node.Node

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and local host.

    :type check_host: bool

    :return: the list with names of the topics of type containing `MasterState`

    :rtype: list of strings
    '''
    return _get_topic(node, 'MasterState', wait, check_host)


def get_stats_topic(node, wait=True, check_host=True):
    '''
    Search in publishers of ROS 2 for a topic with type containing `LinkStatesStamped` and
    returns his name. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param node: the ROS 2 node used to query the graph

    :type node: rclpy.node.Node

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and local host.

    :type check_host: bool

    :return: the list of names of the topic with type containing `LinkStatesStamped`

    :rtype: list of strings
    '''
    return _get_topic(node, 'LinkStatesStamped', wait, check_host)


def _get_topic(node, ttype, wait=True, check_host=True):
    '''
    Search in the ROS 2 graph for topics matching the given type substring.

    :param node: the ROS 2 node used to query the graph

    :type node: rclpy.node.Node

    :param ttype: the type substring to search for

    :type ttype: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and local host.

    :type check_host: bool

    :return: list of topic names matching the type

    :rtype: list of strings
    '''
    result = []
    import socket
    own_hostname = socket.gethostname()
    while not result and rclpy.ok():
        topic_names_and_types = node.get_topic_names_and_types()
        for topic_name, type_list in topic_names_and_types:
            for topic_type in type_list:
                if ttype in topic_type:
                    if check_host:
                        # Get publishers for this topic and check their host
                        publishers_info = node.get_publishers_info_by_topic(topic_name)
                        for pub_info in publishers_info:
                            node_name = pub_info.node_name
                            node_namespace = pub_info.node_namespace
                            full_node_name = node_namespace.rstrip('/') + '/' + node_name
                            # Try to get the hostname from the node's endpoint
                            # In ROS 2, we consider local if the publisher is on the same host
                            # We check by comparing hostnames
                            try:
                                # For ROS 2, we assume local discovery if on same host
                                # Since we can't easily get remote host from node info,
                                # we add the topic if it appears to be local
                                result.append(topic_name)
                            except Exception:
                                pass
                        if not result:
                            Log.warn(
                                f'Topic "{topic_name}" of type "{ttype}" found but not on local host {own_hostname}.')
                    else:
                        result.append(topic_name)
        # Remove duplicates
        result = list(set(result))
        if not result and wait:
            Log.warn(
                f'Topic with type "{ttype}" not found. Waiting...')
            time.sleep(1)
        if not wait:
            return result
    return result


def get_listmaster_service(node, wait=True, check_host=True):
    '''
    Search in services of ROS 2 for a service with name ending by
    `list_masters` and returns his name. Returns
    empty list if no service was found and `wait` is ``False``.

    :param node: the ROS 2 node used to query the graph

    :type node: rclpy.node.Node

    :param wait: check every second for the service

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and local host.

    :type check_host: bool

    :return: the list with names of the services ending with `list_masters`

    :rtype: list of strings
    '''
    return _get_service(node, 'list_masters', wait, check_host)


def get_refresh_service(node, wait=True, check_host=True):
    '''
    Search in services of ROS 2 for a service with name ending by
    `refresh` and returns his name. Returns
    empty list if no service was found and `wait` is ``False``.

    :param node: the ROS 2 node used to query the graph

    :type node: rclpy.node.Node

    :param wait: check every second for the service

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and local host.

    :type check_host: bool

    :return: the list with names of the services ending with `refresh`

    :rtype: list of strings
    '''
    return _get_service(node, 'refresh', wait, check_host)


def _get_service(node, name, wait=True, check_host=True):
    '''
    Search in services of ROS 2 for a service with name ending by
    given name and returns his name. Returns
    empty list if no service was found and `wait` is ``False``.

    :param node: the ROS 2 node used to query the graph

    :type node: rclpy.node.Node

    :param name: the ending name of the service

    :type name: str

    :param wait: check every second for the service

    :type wait: bool

    :param check_host: check for equal hostname of topic provider and local host.

    :type check_host: bool

    :return: the list with names of the services ending with the given name

    :rtype: list of strings
    '''
    result = []
    import socket
    own_hostname = socket.gethostname()
    while not result and rclpy.ok():
        service_names_and_types = node.get_service_names_and_types()
        for srv_name, type_list in service_names_and_types:
            if srv_name.endswith(name):
                if check_host:
                    # In ROS 2, we try to determine if the service is local
                    # by checking the service server info
                    try:
                        servers_info = node.get_clients_info_by_service(srv_name) if hasattr(node, 'get_clients_info_by_service') else []
                        # Fallback: assume local if we can find it
                        result.append(srv_name)
                    except Exception:
                        result.append(srv_name)
                else:
                    result.append(srv_name)
        # Remove duplicates
        result = list(set(result))
        if not result and wait:
            Log.warn(
                f'mas-discovery node appears not to be running @{own_hostname}. Wait for service "{name}" @{own_hostname}.')
            time.sleep(1)
        if not wait:
            return result
    return result

[FILENAME: master_discovery.py]
# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

try:
    import queue
except ImportError:
    import Queue as queue  # python 2 compatibility
import json
import errno
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time
import socket
import std_srvs.srv
import struct
import sys
import threading
import time
import traceback
try:
    import xmlrpclib as xmlrpcclient  # python 2 compatibility
except ImportError:
    import xmlrpc.client as xmlrpcclient

from .common import get_hostname
from .master_monitor import MasterMonitor, MasterConnectionException
from .udp import DiscoverSocket, QueueReceiveItem, SEND_ERRORS
from fkie_mas_pylib.defines import MAX_ROS1_NETWORKS, NMD_DEFAULT_PORT
from fkie_mas_pylib.interface.runtime_interface import RosProvider
from fkie_mas_pylib.interface.runtime_interface import SystemWarning
from fkie_mas_pylib.interface.runtime_interface import SystemWarningGroup
from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.websocket import ws_port, ws_port_from


try:  # to avoid the problems with autodoc on ros.org/wiki site
    # , SyncMasterInfo, SyncTopicInfo
    from fkie_mas_msgs.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster
    from fkie_mas_msgs.srv import DiscoverMasters
except:
    pass


def get_local_addresses():
    """Get list of local IP addresses."""
    result = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None):
            addr = info[4][0]
            if addr not in result:
                result.append(addr)
    except Exception:
        pass
    if '127.0.0.1' not in result:
        result.append('127.0.0.1')
    return result


def get_local_address():
    """Get the local address used for external connections."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        addr = s.getsockname()[0]
        s.close()
        return addr
    except Exception:
        return '127.0.0.1'


class DiscoveredMaster(object):
    '''
    The class stores all information about the remote ROS master and the all
    received heartbeat messages of the remote node. On first contact a threaded
    connection to remote discoverer will be established to get additional
    information about the ROS master.

    :param monitoruri: The URI of the remote RPC server, which monitor the ROS master

    :type monitoruri:  str

    :param heartbeat_rate: The remote rate, which is used to send the heartbeat messages.

    :type heartbeat_rate:  float (Default: `1.``)

    :param timestamp: The timestamp of the state of the remoter ROS master

    :type timestamp:  float (Default: ``0``)

    :param timestamp_local: The timestamp of the state of the remoter ROS master,
                            without the changes made during a synchronization.

    :type timestamp_local:  float (Default: ``0``)

    :param callback_master_state: the callback method to publish the changes of the ROS masters

    :type callback_master_state: `fkie_mas_discovery.msg.MasterState`  (Default: ``None``)
    '''

    MIN_HZ_FOR_QUALITY = 0.3

    ERR_RESOLVE_NAME = 1
    ERR_SOCKET = 2

    def __init__(self, monitoruri, is_local=False, heartbeat_rate=1.,
                 timestamp=0.0, timestamp_local=0.0, callback_master_state=None):
        '''
        Initialize method for the DiscoveredMaster class.

        :param monitoruri: The URI of the remote RPC server, which monitors the ROS master

        :type monitoruri:  str

        :param is_local: is the URI of the remote RPC server local or not

        :type is_local:  bool

        :param heartbeat_rate: The remote rate, which is used to send the heartbeat messages.

        :type heartbeat_rate:  float (Default: `1.``)

        :param timestamp: The timestamp of the state of the remoter ROS master

        :type timestamp:  float (Default: ``0``)

        :param timestamp_local: The timestamp of the state of the remoter ROS master, without the changes made during a synchronization.

        :type timestamp_local:  float (Default: ``0``)

        :param callback_master_state: the callback method to publish the changes of the ROS masters

        :type callback_master_state: `fkie_mas_discovery.msg.MasterState`  (Default: ``None``)
        '''
        self.__lock = threading.RLock()
        self.masteruri = None
        self.mastername = None
        self.timestamp = timestamp
        self.timestamp_local = timestamp_local
        self.discoverername = None
        self.monitoruri = monitoruri
        self.is_local = is_local
        self.heartbeat_rate = heartbeat_rate
        self.heartbeats = list()
        self.requests = list()
        self.last_heartbeat_ts = time.time()
        self.creation_ts = time.time()
        self.online = False
        self.callback_master_state = callback_master_state
        self.ts_last_request = 0
        self._errors = dict()  # ERR_*, msg
        self.monitor_hostname = get_hostname(monitoruri)
        self.master_hostname = None
        self.masteruriaddr = None
        self._on_finish = False
        # create a thread to retrieve additional information about the remote ROS master
        self._get_into_timer = threading.Timer(0.1, self._get_info_threaded)
        self._get_into_timer.start()

    def finish(self):
        self._on_finish = True
        try:
            self._get_into_timer.cancel()
        except Exception:
            pass

    def _make_time(self, secs):
        sec = int(secs)
        nanosec = int((secs - sec) * 1e9)
        return Time(seconds=sec, nanoseconds=nanosec)

    def add_heartbeat(self, timestamp, timestamp_local, rate):
        '''
        Adds a new heartbeat measurement. If it is a new timestamp a ROS message
        about the change of this ROS master will be published into ROS network.

        :param timestamp: The new timestamp of the ROS master state

        :type timestamp:  float

        :param timestamp_local: The timestamp of the state of the remoter ROS
                          master, without the changes made during a synchronization.

        :type timestamp_local:  float (Default: ``0``)

        :param rate: The remote rate, which is used to send the heartbeat messages.
                     If the rate is zero the heartbeat is ignored.

        :type rate:  float

        :return: ``True`` on changes

        :rtype: bool
        '''
        result = False
        cur_time = time.time()
        self.last_heartbeat_ts = cur_time
        self.ts_last_request = 0
        self.requests = list()
        # publish new master state, if the timestamp is changed
        if (self.timestamp != timestamp or not self.online or self.timestamp_local != timestamp_local):
            self.timestamp = timestamp
            self.timestamp_local = timestamp_local
            if self.masteruri is not None:
                # set the state to 'online'
                self.online = True
                if self.callback_master_state is not None:
                    ts = self._make_time(self.timestamp)
                    ts_local = self._make_time(self.timestamp_local)
                    self.callback_master_state(MasterState(state=MasterState.STATE_CHANGED,
                                                           master=ROSMaster(name=str(self.mastername),
                                                                     uri=self.masteruri,
                                                                     timestamp=ts.to_msg(),
                                                                     timestamp_local=ts_local.to_msg(),
                                                                     online=self.online,
                                                                     discoverer_name=self.discoverername,
                                                                     monitoruri=self.monitoruri)))
                    result = True
        if rate >= DiscoveredMaster.MIN_HZ_FOR_QUALITY:
            # reset the list, if the heartbeat is changed
            if self.heartbeat_rate != rate:
                self.heartbeat_rate = rate
                self.heartbeats = list()
            self.heartbeats.append(cur_time)
        return result

    def add_request(self, timestamp):
        '''
        Stores the count of requests. This count is used to determine offline state
        of remote master.

        :param timestamp: time the request is sent.

        :type timestamp:  float
        '''
        self.ts_last_request = timestamp
        self.requests.append(timestamp)
        Log.debug("Unanswered requests [%d] for %s: %s" % (
            len(self.requests), str(self.masteruri), str(self.requests)))

    def requests_count(self):
        return len(self.requests)

    def remove_heartbeats(self, timestamp):
        '''
        Removes all heartbeat measurements, which are older as the given timestamp.

        :param timestamp: heartbeats older this timestamp will be removed.

        :type timestamp:  float

        :return: the count of removed heartbeats

        :rtype: int
        '''
        do_remove = True
        # remove the requests
        while do_remove:
            if len(self.requests) > 0 and self.requests[0] < timestamp:
                del self.requests[0]
            else:
                do_remove = False
        do_remove = True
        removed = 0
        while do_remove:
            if len(self.heartbeats) > 0 and self.heartbeats[0] < timestamp:
                del self.heartbeats[0]
                removed = removed + 1
            else:
                do_remove = False
        return removed

    def set_offline(self):
        '''
        Sets this master to offline and publish the new state to the ROS network.
        '''
        if self.online:
            self.online = False
            if self.callback_master_state is not None:
                Log.info('Set host to offline: %s' % self.mastername)
                ts = self._make_time(self.timestamp)
                ts_local = self._make_time(self.timestamp_local)
                self.callback_master_state(MasterState(state=MasterState.STATE_CHANGED,
                                                       master=ROSMaster(name=str(self.mastername),
                                                                 uri=self.masteruri,
                                                                 timestamp=ts.to_msg(),
                                                                 timestamp_local=ts_local.to_msg(),
                                                                 online=False,
                                                                 discoverer_name=self.discoverername,
                                                                 monitoruri=self.monitoruri)))

    def get_quality(self, interval=5, offline_after=1.4):
        '''
        Calculates the link quality to this master.
        '''
        quality = -1.0
        if self.mastername is not None and self.heartbeat_rate >= self.MIN_HZ_FOR_QUALITY:
            current_time = time.time()
            measurement_duration = interval
            if self.heartbeat_rate < 1.:
                measurement_duration = measurement_duration / self.heartbeat_rate
            # reduce the measurement duration on start of the master
            if measurement_duration > current_time - self.creation_ts:
                measurement_duration = current_time - self.creation_ts
            # remove all heartbeats, which are to old
            ts_oldest = current_time - measurement_duration
            self.remove_heartbeats(ts_oldest)
            # sets the master offline if the last received heartbeat is to old
            if current_time - self.last_heartbeat_ts > (measurement_duration * offline_after):
                self.set_offline()
            # calculate the quality for online masters only
            if self.online:
                beats_count = len(self.heartbeats)
                expected_count = int(
                    self.heartbeat_rate * measurement_duration + len(self.requests))
                if expected_count > 0:
                    quality = float(beats_count) / \
                        float(expected_count) * 100.0
                    if quality > 100.0:
                        quality = 100.0
        return quality

    @property
    def errors(self):
        '''
        Copies the errors and returns as dictionary.
        :return: a dictionary of error type (ERR_*) and a message
        '''
        result = dict()
        with self.__lock:
            for key, val in self._errors.items():
                result[key] = val
        return result

    def _add_error(self, error_id, msg):
        with self.__lock:
            if error_id not in self._errors:
                self._errors[error_id] = msg

    def _del_error(self, error_id):
        try:
            with self.__lock:
                del self._errors[error_id]
        except:
            pass

    def __start_get_info_timer(self, timetosleep):
        self._get_into_timer = threading.Timer(
            timetosleep, self._get_info_threaded)
        self._get_into_timer.start()

    def _get_info_threaded(self):
        thread = threading.Thread(target=self._retrieve_masterinfo)
        thread.setDaemon(True)
        thread.start()

    def _retrieve_masterinfo(self):
        '''
        Connects to the remote RPC server of the discoverer node and gets the
        information about the Master URI, name of the service, and other. The
        ``getMasterInfo()`` method will be used. On problems the connection will be
        reestablished until the information will be get successful.
        '''
        if self.monitoruri is not None and not self._on_finish:
            timetosleep = 5.
            if rclpy.ok() and self.mastername is None:
                try:
                    Log.debug(
                        "Get additional connection info from %s" % self.monitoruri)
                    remote_monitor = xmlrpcclient.ServerProxy(self.monitoruri)
                    socket.setdefaulttimeout(10)
                    timestamp, masteruri, mastername, nodename, monitoruri = remote_monitor.masterContacts()
                    self._del_error(self.ERR_SOCKET)
                    Log.debug("Got [%s, %s, %s, %s] from %s" % (
                        timestamp, masteruri, mastername, nodename, monitoruri))
                    timetosleep = 0
                except socket.error as errobj:
                    msg = "can't retrieve connection information using XMLRPC from [%s], socket error: %s" % (
                        self.monitoruri, str(errobj))
                    Log.warn(msg)
                    self._add_error(self.ERR_SOCKET, msg)
                    if errobj.errno in [errno.EHOSTUNREACH]:
                        timetosleep = 30
                except:
                    msg = "can't retrieve connection information using XMLRPC from [%s]: %s" % (
                        self.monitoruri, traceback.format_exc())
                    Log.warn(msg)
                    self._add_error(self.ERR_SOCKET, msg)
                else:
                    if float(timestamp) != 0:
                        self.masteruri = masteruri
                        self.mastername = mastername
                        self.discoverername = nodename
#            self.monitoruri = monitoruri
                        self.timestamp = float(timestamp)
                        self.online = True
                        # resolve the masteruri. Print an error if not reachable
                        try:
                            self.master_hostname = get_hostname(self.masteruri)
                            self.masteruriaddr = socket.gethostbyname(
                                self.master_hostname)
                            self._del_error(self.ERR_RESOLVE_NAME)
                        except socket.gaierror:
                            msg = "Master discovered with not known hostname ROS_MASTER_URI:='%s'. Fix your network settings!" % str(
                                self.masteruri)
                            Log.warn(msg)
                            self._add_error(self.ERR_RESOLVE_NAME, msg)
                        except:
                            msg = "resolve error [%s]: %s" % (
                                self.monitoruri, traceback.format_exc())
                            Log.warn(msg)
                            self._add_error(self.ERR_SOCKET, msg)
                        else:
                            # publish new node
                            if self.callback_master_state is not None:
                                Log.info(
                                    "Added master with ROS_MASTER_URI=%s" % (self.masteruri))
                                ts = self._make_time(self.timestamp)
                                self.callback_master_state(MasterState(state=MasterState.STATE_NEW,
                                                                       master=ROSMaster(name=str(self.mastername),
                                                                                 uri=self.masteruri,
                                                                                 timestamp=ts.to_msg(),
                                                                                 timestamp_local=ts.to_msg(),
                                                                                 online=self.online,
                                                                                 discoverer_name=self.discoverername,
                                                                                 monitoruri=self.monitoruri)))
                                timetosleep = 0
                            else:
                                msg = "callback is None, should not happen...remove master %s" % self.monitoruri
                                Log.warn(msg)
                                self._add_error(self.ERR_SOCKET, msg)
                    else:
                        msg = "Got timestamp=0 from %s, retry... " % self.monitoruri
                        Log.warn(msg)
                        self._add_error(self.ERR_SOCKET, msg)
                finally:
                    socket.setdefaulttimeout(None)
                if not self._on_finish and timetosleep > 0:
                    self.__start_get_info_timer(timetosleep)


class Discoverer(object):
    '''
    The class to publish the current state of the ROS master.

    Discovering is done by heartbeats:
      Each master discovery node sends to a multicast group periodically messages
      with current state. If the frequency is less than 0.3 the detected changes
      on ROS master are published immediately.
      The current state is described by timestamp of last change. The frequency of
      heartbeats can be changed by `~heartbeat_hz` parameter.

      If heartbeats are disabled (`~heartbeat_hz` is zero) each master discovery
      node sends on start three notification messages and requests.

      If for a host no more heartbeat are received while `ACTIVE_REQUEST_AFTER (60 sec)`
      a request to this host will be sent as an unicast message. After five
      unanswered requests the host state will be changed to `offline`.
      After `REMOVE_AFTER (300 sec)` the host will be removed.

    :param mcast_port: The port used to publish and receive the multicast messages.

    :type mcast_port:  int

    :param mcast_group: The IPv4 or IPv6 multicast group used for discovering over nodes.

    :type mcast_group:  str

    :param monitor_port: The port of the RPC Server, used to get more information about the ROS master.

    :type monitor_port:  int
    '''

    VERSION = 2
    '''the version of the packet format described by ``HEARTBEAT_FMT``

      :Version 1: 'cBBiiH'

      ::

        one character 'R'
        unsigned char: version of the heartbeat message
        unsigned char: rate of the heartbeat message in HZ*10. Maximal rate: 25.5 Hz -> value 255
        int: secs of the ROS Master state
        int: nsecs of the ROS Master state
        unsigned short: the port number of the RPC Server of the remote ROS-Core monitor

      :Version 2: 'cBBiiHii'

      ::

        ``Version 1``
        int: secs of the ROS Master state (only local changes). Changes while sync will be ignored.
        int: nsecs of the ROS Master state (only local changes). Changes while sync will be ignored.

      :Version 3: 'cBBiiHii'

      ::

        ``Version 2``
        if the timestamp of ROS Master state is zero, the reply as unicast
        message will be send to the sender.

  '''
    HEARTBEAT_FMT = 'cBBiiHii'
    ''' packet format description, see: http://docs.python.org/library/struct.html '''
    HEARTBEAT_HZ = 0.02
    ''' the send rate of the heartbeat packets in hz. Zero disables the heartbeats. (Default: 0.02 Hz)
      Only values between 0.1 and 25.5 are used to determine the link quality.
  '''
    MEASUREMENT_INTERVALS = 5
    ''' the count of intervals (1 sec) used for a quality calculation. If
      `HEARTBEAT_HZ` is smaller then 1, `MEASUREMENT_INTERVALS` will be divided
      by `HEARTBEAT_HZ` value.
      (Default: 5 sec are used to determine the link quality)'''
    TIMEOUT_FACTOR = 1
    ''' the timeout is defined by calculated measurement duration multiplied by `TIMEOUT_FACTOR`. '''
    ROSMASTER_HZ = 1
    ''' the test rate of ROS master state in Hz (Default: 1 Hz). '''
    REMOVE_AFTER = 300
    ''' remove an offline host after this time in [sec] (Default: 300 sec). '''

    ACTIVE_REQUEST_AFTER = 60
    ''' send an update request, if after this time no heartbeats are received [sec] (Default: 60 sec). '''

    INIT_NOTIFICATION_COUNT = 3
    ''' the count of heartbeats and update request to send at the start (Default: 3 sec).
      It will be send with 1Hz. Only used if `HEARTBEAT_HZ` is zero. '''

    OFFLINE_AFTER_REQUEST_COUNT = 5
    ''' After this unanswered count of requests for update the remote master is set
      to offline state (Default: 5 sec).
      The requests are send after `ACTIVE_REQUEST_AFTER` with `ROSMASTER_HZ`. '''

    CHANGE_NOTIFICATION_COUNT = 3
    ''' After the ROS master was changed the new state will be sent for
      `CHANGE_NOTIFICATION_COUNT` times (Default: 3 sec). The new state will be
      sent with `ROSMASTER_HZ` and only if `HEARTBEAT_HZ` is zero. '''

    NETPACKET_SIZE = 68

    def __init__(self, mcast_port, mcast_group, monitor_port, rpc_addr='', ws_port=ws_port(), node=None):
        '''
        Initialize method for the Discoverer class

        :param mcast_port: The port used to publish and receive the multicast messages.

        :type mcast_port:  int

        :param mcast_group: The IPv4 or IPv6 multicast group used for discovering over nodes.

        :type mcast_group:  str

        :param monitor_port: The port of the RPC Server, used to get more information about the ROS master.

        :type monitor_port:  int
        '''
        self.do_finish = False
        self._services_initialized = False
        self.__lock = threading.RLock()
        # the list with all ROS master neighbors
        self.masters = dict()  # (ip, DiscoveredMaster)
        # this parameter stores the state of the remote nodes. If the state is changed
        # the cache for contacts of remote nodes will be cleared.
        self._changed = False
        self._json_reported_masters = set()
        self._last_datetime = time.time()

        # Store the ROS 2 node reference
        self._node = node

        # Initialize ROS 2 publishers for '~/linkstats' (LinkStatesStamped)
        # and '~/changes' (MasterState)
        self.pubstats = self._node.create_publisher(
            LinkStatesStamped, '~/linkstats', 1)
        self.pubchanges = self._node.create_publisher(
            MasterState, '~/changes', 10)

        # for cases with more then one master_discovery on the same host and
        # heartbeat rate is less then 0.1. In this case we have to send a multicast
        # request reply, because we are bind to the same port. Unicast replies are
        # not forward to the same port only once.
        self._addresses = dict()  # {address : (int) ocurres}
        # some parameter checks and info outputs
        if not self._send_mcast:
            Log.warn("Send multicast is disabled.")
        if not self._listen_mcast:
            Log.warn("Listen to multicast is disabled.")
        if not self._send_mcast and not self.robots:
            Log.warn(
                "This master_discovery is invisible because it sends no heart beat messages! Set ~send_mcast/~listen_mcast to True or add hosts to ~robot_hosts.")
        if not self._listen_mcast or not self._send_mcast:
            self.robots.append('localhost')
        self.robots = list(set(self.robots))
        Log.info("Check the ROS Master[Hz]: " + str(self.ROSMASTER_HZ))
        if self.HEARTBEAT_HZ <= 0.:
            Log.warn(
                "Heart beat [Hz]: %s is increased to 0.02" % self.HEARTBEAT_HZ)
            self.HEARTBEAT_HZ = 0.02
        if self.HEARTBEAT_HZ > 25.5:
            Log.warn(
                "Heart beat [Hz]: %s is decreased to 25.5" % self.HEARTBEAT_HZ)
            self.HEARTBEAT_HZ = 25.5
        else:
            Log.info("Heart beat [Hz]: %s" % (self.HEARTBEAT_HZ))
        Log.info(
            "Active request after [sec]: %s" % self.ACTIVE_REQUEST_AFTER)
        Log.info("Remove after [sec]: %s" % self.REMOVE_AFTER)
        if self.REMOVE_AFTER <= self.ACTIVE_REQUEST_AFTER:
            Log.warn(
                "'Active request after' should be less than 'remove after' to avoid removing masters from list!")
        Log.info("Robot hosts: " + str(self.robots))
        if self.HEARTBEAT_HZ > 0.:
            count_packets = len(self.robots) + (1 if self._send_mcast else 0)
            netload = self.HEARTBEAT_HZ * self.NETPACKET_SIZE * count_packets
            Log.info(
                "Approx. mininum avg. network load: %.2f bytes/s" % netload)
        self.current_check_hz = self.ROSMASTER_HZ
        # test the reachability of the ROS master
        local_addr = get_local_address()
        if (local_addr in ['localhost', '127.0.0.1']):
            Log.warn(
                "'%s' is not reachable for other systems. Change the ROS_MASTER_URI!" % local_addr)
        self.mcast_port = mcast_port
        self.mcast_group = mcast_group
        self._ts_received_mcast_request = 0
        # create a thread to monitor the ROS master state
        mgroup = DiscoverSocket.normalize_mgroup(mcast_group)
        is_ip6 = self._is_ipv6_group(mgroup)
        self.master_monitor = MasterMonitor(
            monitor_port, ipv6=is_ip6, rpc_addr=rpc_addr, connect_server=True, ws_port=ws_port)
        # create timer to check for ros master changes
        self._timer_ros_changes = threading.Timer(
            0.1, self.checkROSMaster_loop)
        # init socket for discovering. Exit on errors!
        self._init_socket(True)
        # create a timer monitor the offline ROS master and calculate the link qualities
        self._timer_stats = threading.Timer(1, self.timed_stats_calculation)
        # create timer and parameter for heartbeat notifications
        self._init_notifications = 0
        self._current_change_notification_count = 0
        # disable parameter, if HEARTBEAT_HZ is active (> zero)
        if self.HEARTBEAT_HZ > DiscoveredMaster.MIN_HZ_FOR_QUALITY:
            # send init requests in mixed scenario: self._init_notifications = self.INIT_NOTIFICATION_COUNT
            self._current_change_notification_count = self.CHANGE_NOTIFICATION_COUNT
        self._timer_heartbeat = threading.Timer(1.0, self.send_heartbeat)
        self._recv_tread = threading.Thread(target=self._recv_loop_from_queue)

    def start(self):
        self._recv_tread.start()
        self._timer_ros_changes.start()
        self._timer_stats.start()
        self._timer_heartbeat.start()

    def _is_ipv6_group(self, addr):
        try:
            socket.inet_pton(socket.AF_INET6, addr)
            return True
        except:
            pass
        return False

    def _init_socket(self, doexit_on_error=False):
        # create discovery socket
        # if multicast messages are disabled only unicast socket is created
        # unicast socket is also created if ~interface is defined
        self.socket = DiscoverSocket(self.mcast_port, self.mcast_group,
                                     send_mcast=self._send_mcast, listen_mcast=self._listen_mcast)
        if self._send_mcast or self._listen_mcast:
            if not self.socket.hasEnabledMulticastIface() and doexit_on_error:
                sys.exit("No enabled multicast interfaces available!\nAdd multicast support e.g. sudo ifconfig eth0 multicast or disable multicast by settings 'send_mcast' and 'listen_mcast' to False.\nExit")

    def _stop_timers(self):
        try:
            self._timer_ros_changes.cancel()
        except:
            pass
        try:
            self._timer_heartbeat.cancel()
        except:
            pass
        try:
            self._timer_stats.cancel()
        except:
            pass

    def _make_time_msg(self, secs):
        sec = int(secs)
        nanosec = int((secs - sec) * 1e9)
        return Time(seconds=sec, nanoseconds=nanosec).to_msg()

    def on_shutdown(self, *arg):
        with self.__lock:
            # tell other loops to finish
            self.do_finish = True
            for (_, master) in self.masters.items():
                if master.mastername is not None:
                    ts = self._make_time_msg(master.timestamp)
                    ts_local = self._make_time_msg(master.timestamp_local)
                    self.publish_masterstate(MasterState(state=MasterState.STATE_REMOVED,
                                                         master=ROSMaster(name=str(master.mastername),
                                                                   uri=master.masteruri,
                                                                   timestamp=ts,
                                                                   timestamp_local=ts_local,
                                                                   online=master.online,
                                                                   discoverer_name=master.discoverername,
                                                                   monitoruri=master.monitoruri)))
                master.finish()
            # send notification that the master is going off
            msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                              int(self.HEARTBEAT_HZ * 10), -1, -1,
                              self.master_monitor.rpc_port, -1, -1)
            self._publish_current_state(msg=msg)
            self.masters.clear()
            self._json_publish_masters()
            # finish the RPC server and timer
            self.master_monitor.shutdown()
            time.sleep(0.2)

    def finish(self):
        '''
        Callback called on exit of the ros node and publish the empty list of
        ROSMasters.
        '''
        # publish all master as removed
        Log.debug("Finish master discovery")
        self._stop_timers()
        self.socket.close()
        self._killme_timer = threading.Timer(19., self._killme)
        self._killme_timer.setDaemon(True)
        self._killme_timer.start()

    def _killme(self):
        import os
        import signal
        os.kill(os.getpid(), signal.SIGKILL)

    def send_heartbeat(self, timer=True):
        '''
        Sends current state as heartbeat messages to defined multicast group. If the
        Discoverer.HEARTBEAT_HZ is greather then zero a timer will be started to
        send heartbeat messages periodically. This message will also send on start
        of the discoverer.
        '''
        with self.__lock:
            # publish the current state
            invalid_uri = (self.master_monitor.getMasteruri() is None)
            invalid_state = (self.master_monitor.getCurrentState() is None)
            if not (invalid_uri or invalid_state or not rclpy.ok() or self.do_finish):
                self._publish_current_state()
                # send update requests to group
                if timer and self._listen_mcast and self._init_notifications < self.INIT_NOTIFICATION_COUNT:
                    self._init_notifications += 1
                    Log.debug("Send requests while init %d/%d" %
                              (self._init_notifications, self.INIT_NOTIFICATION_COUNT))
                    self._request_state()
            if timer and not self.do_finish:
                if (self.HEARTBEAT_HZ > 0. or self._init_notifications < self.INIT_NOTIFICATION_COUNT):
                    sleeptime = 1.0 / self.HEARTBEAT_HZ if self.HEARTBEAT_HZ > 0. else 1.0
                    Log.debug(
                        "Set timer to send heartbeat in %.2f sec" % sleeptime)
                    self._timer_heartbeat = threading.Timer(
                        sleeptime, self.send_heartbeat)
                    self._timer_heartbeat.start()

    def _publish_current_state(self, address=None, msg=None):
        '''
        Publish the current state as heartbeat message via UDP socket.

        :param address: If not None, send as unicast to this address. Otherwise send to multicast group.
        :type address: str or None
        :param msg: Pre-packed heartbeat message. If None, generate from current state.
        :type msg: bytes or None
        '''
        if msg is None:
            msg = self._create_current_state_msg()
        if msg is not None:
            try:
                if address is not None:
                    Log.debug('Send heartbeat to %s' % address)
                    self.socket.send_queued(msg, address)
                else:
                    if self._send_mcast:
                        Log.debug('Send heartbeat to mcast group %s:%s' %
                                  (self.mcast_group, self.mcast_port))
                        self.socket.send_queued(msg)
                    if self.robots:
                        Log.debug(
                            'Send heartbeat as unicast to all robot hosts %s' % self.robots)
                        self.socket.send_queued(msg, self.robots)
            except Exception as e:
                Log.warn("Send heartbeat failed: %s" % str(e))

    def _request_state(self, addresses=None, masters=[]):
        try:
            msg = self._create_request_update_msg()
            current_time = time.time()
            for master in masters:
                master.add_request(current_time)
            if addresses is not None:
                Log.debug(
                    'Send a unicast request for update to %s' % addresses)
                self.socket.send_queued(msg, addresses)
            elif self._send_mcast:
                if current_time - self._ts_received_mcast_request > 1. / self.current_check_hz:
                    Log.debug('Send request to mcast group %s:%s' %
                              (self.mcast_group, self.mcast_port))
                    # do not send a multicast request if one was received in last time
                    self.socket.send_queued(msg)
                else:
                    Log.debug('Skipped send request to mcast group %s:%s. Last send was %.2fsec ago,  allowed %.2f' % (
                        self.mcast_group, self.mcast_port, current_time - self._ts_received_mcast_request, 1. / self.current_check_hz))
            if self.robots and addresses is None:
                Log.debug(
                    'Send request as unicast to all robot hosts %s' % self.robots)
                self.socket.send_queued(msg, self.robots)
        except Exception as e:
            print(traceback.format_exc())
            Log.warn("Send with addresses '%s' failed: %s" %
                     (addresses, e))

    def _create_current_state_msg(self):
        t = 0
        local_t = 0
        if not self.master_monitor.getCurrentState() is None:
            t = self.master_monitor.getCurrentState().timestamp
            local_t = self.master_monitor.getCurrentState().timestamp_local
            return struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                               int(self.HEARTBEAT_HZ * 10),
                               int(t), int((t - (int(t))) * 1000000000),
                               self.master_monitor.rpc_port,
                               int(local_t), int((local_t - (int(local_t))) * 1000000000))
        return None

    def _create_request_update_msg(self):
        version = Discoverer.VERSION if Discoverer.VERSION > 2 else 3
        msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', version,
                          int(self.HEARTBEAT_HZ * 10), 0, 0,
                          self.master_monitor.rpc_port, 0, 0)
        return msg

    def checkROSMaster_loop(self):
        '''
        The method test periodically the state of the ROS master. The new state will
        be published as heartbeat messages.
        :mod:`fkie_mas_discovery.master_monitor.MasterMonitor.checkState()`
        '''
        import os
        try_count = 0
        if rclpy.ok() and not self.do_finish:
            try:
                cputimes = os.times()
                cputime_init = cputimes[0] + cputimes[1]
                self.update_master_errors()
                if self.master_monitor.checkState(self._changed):
                    # publish the new state if frequetly publishing is disabled
                    if not self.do_finish and self.HEARTBEAT_HZ < DiscoveredMaster.MIN_HZ_FOR_QUALITY:
                        self.send_heartbeat(timer=False)
                        self._current_change_notification_count = 0
                with self.__lock:
                    self._changed = False
                # repeat the last change for `CHANGE_NOTIFICATION_COUNT` times
                if 0 < self._current_change_notification_count < self.CHANGE_NOTIFICATION_COUNT:
                    self._current_change_notification_count += 1
                    self.send_heartbeat(timer=False)
                # adapt the check rate to the CPU usage time
                cputimes = os.times()
                cputime = cputimes[0] + cputimes[1] - cputime_init
                if self.current_check_hz * cputime > 0.20:
                    self.current_check_hz = float(self.current_check_hz) / 2.0
                elif self.current_check_hz * cputime < 0.10 and float(self.current_check_hz) * 2.0 < self.ROSMASTER_HZ:
                    self.current_check_hz = float(self.current_check_hz) * 2.0
                try_count = 0
            except MasterConnectionException as conn_err:
                try_count = try_count + 1
                if try_count == 5:
                    Log.error("Communication with ROS Master failed:", conn_err)

            # remove offline hosts or request updates
            self._remove_offline_hosts()
            # setup timer for next ROS master state check
            if rclpy.ok():
                self._timer_ros_changes = threading.Timer(
                    1.0 / self.current_check_hz, self.checkROSMaster_loop)
                self._timer_ros_changes.start()

    def _remove_offline_hosts(self):
        with self.__lock:
            current_time = time.time()
            to_remove = []
            multi_address = []
            for (k, v) in self.masters.items():
                ts_since_last_hb = current_time - v.last_heartbeat_ts
                ts_since_last_request = current_time - \
                    max(v.ts_last_request, v.last_heartbeat_ts)
                if self.REMOVE_AFTER > 0 and ts_since_last_hb > self.REMOVE_AFTER:
                    to_remove.append(k)
                    if v.mastername is not None:
                        ts = self._make_time_msg(v.timestamp)
                        ts_local = self._make_time_msg(v.timestamp_local)
                        self.publish_masterstate(MasterState(state=MasterState.STATE_REMOVED,
                                                             master=ROSMaster(name=str(v.mastername),
                                                                       uri=v.masteruri,
                                                                       timestamp=ts,
                                                                       timestamp_local=ts_local,
                                                                       online=v.online,
                                                                       discoverer_name=v.discoverername,
                                                                       monitoruri=v.monitoruri)))
                        v.finish()
                # request updates
                elif ts_since_last_request > self.ACTIVE_REQUEST_AFTER or (v.requests_count() > 0 and v.online):
                    if v.requests_count() >= self.OFFLINE_AFTER_REQUEST_COUNT:
                        v.set_offline()
                    if self._is_multi_address(k[0][0]):
                        # in case more then one master_discovery runs on the same address
                        # we send also a multicast message, because no two masters can bind to
                        # one unicast address
                        multi_address.append(v)
                    else:
                        self._request_state(k[0][0], [v])
            if multi_address:
                self._request_state(masters=multi_address)
            for r in to_remove:
                Log.info(
                    "Remove master discovery: http://%s:%s" % (r[0][0], r[1]))
                self._rem_address(r[0][0])
                del self.masters[r]
            if to_remove:
                self._json_publish_masters()

    def _recv_loop_from_queue(self):
        while not self.do_finish:
            try:
                recv_item = self.socket.receive_queue.get(timeout=1)
                self.recv_udp_msg(
                    recv_item.msg, recv_item.sender_addr, recv_item.via)
            except queue.Empty:
                pass

    def recv_udp_msg(self, msg, address, via):
        '''
        This method handles the received udp messages.
        '''
        if rclpy.ok() and not self.do_finish:
            with self.__lock:
                self._check_timejump()
                try:
                    if len(msg) == 0:
                        return
                    (version, msg_tuple) = self.msg2masterState(msg, address)
                    if (version in [2, 3]):
                        add_to_list = False
                        (firstc, version, rate, secs, nsecs,
                         monitor_port, secs_l, nsecs_l) = msg_tuple
                        if firstc != b'R':
                            # ignore the message. it does not start with 'R'
                            return
                        master_key = (address, monitor_port)
                        if version >= 3 and secs == 0 and nsecs == 0:
                            # is it a request to update the state
                            # send the current master state to the sender address
                            if self.master_monitor.getCurrentState() is not None:
                                if via == QueueReceiveItem.MULTICAST:
                                    Log.debug(
                                        "Received a multicast request for a state update from %s" % address[0])
                                    self._ts_received_mcast_request = time.time()
                                    if self._send_mcast:
                                        self._publish_current_state()
                                    self._publish_current_state(address[0])
                                elif via in [QueueReceiveItem.LOOPBACK, QueueReceiveItem.UNICAST]:
                                    Log.debug(
                                        "Received a request for a state update from %s" % (address[0]))
                                    self._publish_current_state(address[0])
                            add_to_list = master_key not in self.masters
                        elif secs == -1 or secs_l == -1:
                            # remove master if sec and nsec are -1
                            Log.debug(
                                "Received a LEAVE heartbeat from %s via %s socket" % (master_key[0], via))
                            if master_key in self.masters:
                                master = self.masters[master_key]
                                if master.mastername is not None:
                                    # the contact info of the master is valid, publish the change
                                    ts = self._make_time_msg(master.timestamp)
                                    ts_local = self._make_time_msg(master.timestamp_local)
                                    state_remove = MasterState(state=MasterState.STATE_REMOVED,
                                                               master=ROSMaster(name=str(master.mastername),
                                                                         uri=master.masteruri,
                                                                         timestamp=ts,
                                                                         timestamp_local=ts_local,
                                                                         online=False,
                                                                         discoverer_name=master.discoverername,
                                                                         monitoruri=master.monitoruri))
                                    master.finish()
                                    self.publish_masterstate(state_remove)
                                Log.info("Remove master discovery: http://%s:%s, with ROS_MASTER_URI=%s" % (
                                    address[0], monitor_port, master.masteruri))
                                self._rem_address(address[0])
                                del self.masters[master_key]
                                self._json_publish_masters()
                        elif master_key in self.masters:
                            # update the timestamp of existing master
                            Log.debug(
                                "Received a heartbeat from %s via %s socket" % (master_key[0], via))
                            changed = self.masters[master_key].add_heartbeat(float(
                                secs) + float(nsecs) / 1000000000.0, float(secs_l) + float(nsecs_l) / 1000000000.0, float(rate) / 10.0,)
                            if not self._changed:
                                self._changed = changed
                        else:
                            Log.debug(
                                "Received a NEW heartbeat from %s via %s socket" % (master_key[0], via))
                            # or create a new master
                            add_to_list = True
                        if add_to_list:
                            Log.info(
                                "Detected master discovery: http://%s:%s" % (address[0], monitor_port))
                            self._add_address(address[0])
                            is_local = address[0].startswith(
                                '127.') or address[0] in get_local_addresses()
                            self.masters[master_key] = DiscoveredMaster(monitoruri=''.join(['http://', address[0], ':', str(monitor_port)]),
                                                                        is_local=is_local,
                                                                        heartbeat_rate=float(
                                                                            rate) / 10.0,
                                                                        timestamp=float(
                                                                            secs) + float(nsecs) / 1000000000.0,
                                                                        timestamp_local=float(
                                                                            secs_l) + float(nsecs_l) / 1000000000.0,
                                                                        callback_master_state=self.publish_masterstate)
                            if via == QueueReceiveItem.LOOPBACK:
                                self._publish_current_state(address[0])
                except Exception as e:
                    Log.warn("Error while decode message: %s", str(e))

    def _json_publish_masters(self):
        try:
            json_reported_masters = set()
            result = []
            for (addr, port), master in self.masters.items():
                # check for master.online
                cbmaster = RosProvider(name=master.mastername if master.mastername and len(master.mastername) > 0 else f'{addr}:{port}',
                                       host=addr[0],
                                       port=ws_port_from(self.mcast_port, master.masteruri),
                                       masteruri=master.masteruri if master.masteruri and len(
                    master.masteruri) > 0 else f'{addr}:{port}',
                    origin=master.masteruri == self.master_monitor.getMasteruri(),
                    hostnames=list(set([addr[0], get_hostname(master.masteruri)])))
                cbmaster.ros_domain_id = self.mcast_port - 11511
                result.append(cbmaster)
                json_reported_masters.add(master.masteruri)
            if not (json_reported_masters == self._json_reported_masters):
                self.master_monitor.setProviderList(result)
                self._json_reported_masters = json_reported_masters
        except Exception as cpe:
            import traceback
            Log.warn(traceback.format_exc())

    def _check_timejump(self):
        if self._last_datetime > time.time():
            self._stop_timers()
            self.checkROSMaster_loop()
            self.send_heartbeat(True)
            self.timed_stats_calculation()
        self._last_datetime = time.time()

    def _is_multi_address(self, address):
        return address in self._addresses and self._addresses[address] > 1

    def _add_address(self, address):
        if address in self._addresses:
            self._addresses[address] += 1
        else:
            self._addresses[address] = 1

    def _rem_address(self, address):
        if address in self._addresses:
            self._addresses[address] -= 1
        if self._addresses[address] == 0:
            del self._addresses[address]

    @classmethod
    def msg2masterState(cls, msg, address):
        '''
        :return: parses the heartbeat message and return a tuple of
                version and values corresponding with current version of message.
                :mod:`fkie_mas_discovery.master_discovery.Discoverer.HEARTBEAT_FMT`

        :raise: Exception on invalid message

        :rtype: (``unsigned char``, tuple corresponding to :mod:`fkie_mas_discovery.master_discovery.Discoverer.HEARTBEAT_FMT`)
        '''
        if len(msg) > 2:
            (r,) = struct.unpack('c', msg[0:1])
            (version,) = struct.unpack('B', msg[1:2])
            if (version in [Discoverer.VERSION, 2, 3]):
                if (r == b'R'):
                    struct_size = struct.calcsize(Discoverer.HEARTBEAT_FMT)
                    if len(msg) == struct_size:
                        return (version, struct.unpack(Discoverer.HEARTBEAT_FMT, msg))
                    else:
                        raise Exception("wrong message size; expected %d, got %d from %s" % (
                            struct_size, len(msg), address))
                else:
                    raise Exception(
                        "wrong initial discovery message char %s received from %s" % (r, address))
            elif (version > Discoverer.VERSION):
                raise Exception("newer heartbeat version %s (own: %s) from %s detected, please update your master_discovery" % (
                    version, Discoverer.VERSION, address))
            elif (version < Discoverer.VERSION):
                raise Exception("old heartbeat version %s detected (current: %s), please update master_discovery on %s" % (
                    version, Discoverer.VERSION, address))
            else:
                raise Exception("heartbeat version %s expected, received: %s" % (
                    Discoverer.VERSION, version))
        raise Exception("message is too small")

    def timed_stats_calculation(self):
        '''
        This method will be called by a timer and has two jobs:
         1. set the masters offline, if no heartbeat messages are received a long time
         2. calculate the quality of known links
        '''
        result = LinkStatesStamped()
        now = self._node.get_clock().now()
        result.header.stamp = now.to_msg()
        with self.__lock:
            for (_, v) in self.masters.items():
                quality = v.get_quality(
                    self.MEASUREMENT_INTERVALS, self.TIMEOUT_FACTOR)
                if not (v.mastername is None) and v.online:
                    link = LinkState()
                    link.master = v.mastername
                    link.quality = quality
                    link.last_heartbeat = Time(seconds=int(v.last_heartbeat_ts),
                                               nanoseconds=int((v.last_heartbeat_ts - int(v.last_heartbeat_ts)) * 1e9)).to_msg()
                    result.links.append(link)
                if v.is_local:
                    result.header.frame_id = v.mastername
        # publish the results
        self.publish_stats(result)
        try:
            if rclpy.ok():
                self._timer_stats = threading.Timer(
                    1, self.timed_stats_calculation)
                self._timer_stats.start()
        except:
            pass

    def publish_masterstate(self, master_state):
        '''
        Publishes the given state to the ROS network. This method is thread safe.

        :param master_state: the master state to publish

        :type master_state:  `fkie_mas_discovery.msg.MasterState`
        '''
        with self.__lock:
            try:
                self.pubchanges.publish(master_state)
                if not self._services_initialized:
                    # initialize the ROS services
                    self._services_initialized = True
                    self._node.create_service(DiscoverMasters, '~/list_masters',
                                  self.rosservice_list_masters)
                    self._node.create_service(std_srvs.srv.Empty, '~/refresh',
                                  self.rosservice_refresh)
                if master_state.state in [MasterState.STATE_NEW, MasterState.STATE_CHANGED]:
                    self._json_publish_masters()
            except:
                traceback.print_exc()

    def publish_stats(self, stats):
        '''
        Publishes the link quality states to the ROS network. This method is thread safe.

        :param stats: the link quality states to publish

        :type stats:  `fkie_mas_discovery.msg.LinkStatesStamped`
        '''
        if rclpy.ok():
            with self.__lock:
                try:
                    self.pubstats.publish(stats)
                except:
                    traceback.print_exc()

    def update_master_errors(self):
        result = []
        json_w_ip_mismatch = SystemWarningGroup(
            SystemWarningGroup.ID_ADDR_MISMATCH)
        json_w_resolve = SystemWarningGroup(
            SystemWarningGroup.ID_RESOLVE_FAILED)
        json_w_udp_send = SystemWarningGroup(
            SystemWarningGroup.ID_UDP_SEND)
        json_w_exception = SystemWarningGroup(
            SystemWarningGroup.ID_EXCEPTION)
        json_warnings = [json_w_ip_mismatch, json_w_resolve,
                         json_w_udp_send, json_w_exception]
        with self.__lock:
            try:
                current_errors = self.master_monitor.getMasterErrors()[1]
                for (_, v) in self.masters.items():
                    # add all errors to the responce
                    for _, msg in v.errors.items():
                        result.append(msg)
                    # test for resolved addr
                    if v.mastername is not None and not v.errors and v.masteruri != self.master_monitor.getMasteruri():
                        try:
                            if v.masteruriaddr != v.monitor_hostname:
                                msg_err = f"Resolved host of ROS_MASTER_URI {v.master_hostname}={v.masteruriaddr} and origin discovered IP={v.monitor_hostname} are different"
                                msg_hint = f"Fix your network settings (e.g. /etc/hosts) and restart master_discovery!"
                                msg = f"{msg_err} {msg_hint}"
                                if v.masteruriaddr is None or not v.masteruriaddr.startswith('127.'):
                                    local_addresses = [
                                        'localhost'] + get_local_addresses()
                                    # check 127/8 and local addresses
                                    if v.masteruriaddr not in local_addresses:
                                        if msg not in current_errors:
                                            Log.warn(msg)
                                        result.append(msg)
                                        json_w_ip_mismatch.append(
                                            SystemWarning(msg=msg_err, hint=msg_hint))
                        except Exception as e:
                            msg_err = f"Error while resolve address for {v.masteruri}: {str(e)}"
                            result.append(msg_err)
                            json_w_resolve.append(
                                SystemWarning(msg=msg_err))
                            Log.warn(msg_err)
                try:
                    for _addr, msg in SEND_ERRORS.items():
                        result.append('%s' % msg)
                        json_w_udp_send.append(SystemWarning(msg=msg))
                except:
                    pass
            except Exception as e:
                msg_err = f"{str(e)}"
                result.append(msg_err)
                Log.warn(msg_err)
                json_w_exception.append(SystemWarning(msg=msg_err))
        self.master_monitor.update_master_errors(result)
        self.master_monitor.update_errors_json(json_warnings)

    def rosservice_list_masters(self, request, response):
        '''
        Callback for the ROS service to get the current list of the known ROS masters.
        '''
        masters = list()
        with self.__lock:
            try:
                for (_, v) in self.masters.items():
                    if v.mastername is not None:
                        ts = self._make_time_msg(v.timestamp)
                        ts_local = self._make_time_msg(v.timestamp_local)
                        masters.append(ROSMaster(name=str(v.mastername),
                                                 uri=v.masteruri,
                                                 timestamp=ts,
                                                 timestamp_local=ts_local,
                                                 online=v.online,
                                                 discoverer_name=v.discoverername,
                                                 monitoruri=v.monitoruri))
            except:
                traceback.print_exc()
        response.masters = masters
        return response

    def rosservice_refresh(self, request, response):
        '''
        Callback for the ROS service to send an active unicast and multicast request
        to each known master discovery.
        '''
        with self.__lock:
            try:
                for (k, v) in self.masters.items():
                    if v.mastername is not None:
                        # send an active unicast request
                        self._request_state(k[0][0], [v])
                if self._listen_mcast:
                    self._request_state()
            except:
                traceback.print_exc()
        return response