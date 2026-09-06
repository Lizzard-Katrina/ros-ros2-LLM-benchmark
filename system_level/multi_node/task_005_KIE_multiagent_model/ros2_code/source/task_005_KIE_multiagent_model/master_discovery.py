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
import errno
import socket
import struct
import sys
import threading
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

from task_005_KIE_multiagent_model.common import get_hostname

try:  # to avoid the problems with autodoc on ros.org/wiki site
    from fkie_mas_msgs.msg import LinkState, LinkStatesStamped, MasterState, ROSMaster
    from fkie_mas_msgs.srv import DiscoverMasters
except Exception:
    # Define minimal stubs for environments where fkie_mas_msgs is not available
    pass


class _LogCompat:
    """Minimal logging compatibility shim."""
    @staticmethod
    def warn(msg, *args):
        print(f"[WARN] {msg}" % args if args else f"[WARN] {msg}")

    @staticmethod
    def info(msg, *args):
        print(f"[INFO] {msg}" % args if args else f"[INFO] {msg}")

    @staticmethod
    def debug(msg, *args):
        pass

    @staticmethod
    def error(msg, *args):
        print(f"[ERROR] {msg}" % args if args else f"[ERROR] {msg}")


Log = _LogCompat()


class DiscoveredMaster(object):
    '''
    The class stores all information about the remote ROS master and the all
    received heartbeat messages of the remote node.
    '''

    MIN_HZ_FOR_QUALITY = 0.3

    ERR_RESOLVE_NAME = 1
    ERR_SOCKET = 2

    def __init__(self, monitoruri, is_local=False, heartbeat_rate=1.,
                 timestamp=0.0, timestamp_local=0.0, callback_master_state=None):
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
        self._errors = dict()
        self.monitor_hostname = get_hostname(monitoruri)
        self.master_hostname = None
        self.masteruriaddr = None
        self._on_finish = False
        self._get_into_timer = threading.Timer(0.1, self._get_info_threaded)
        self._get_into_timer.start()

    def finish(self):
        self._on_finish = True
        try:
            self._get_into_timer.cancel()
        except Exception:
            pass

    def add_heartbeat(self, timestamp, timestamp_local, rate):
        result = False
        cur_time = time.time()
        self.last_heartbeat_ts = cur_time
        self.ts_last_request = 0
        self.requests = list()
        if (self.timestamp != timestamp or not self.online or self.timestamp_local != timestamp_local):
            self.timestamp = timestamp
            self.timestamp_local = timestamp_local
            if self.masteruri is not None:
                self.online = True
                if self.callback_master_state is not None:
                    clock = Clock(clock_type=ClockType.ROS_TIME)
                    now = clock.now()
                    self.callback_master_state(MasterState(
                        state=MasterState.STATE_CHANGED,
                        master=ROSMaster(
                            name=str(self.mastername),
                            uri=self.masteruri,
                            timestamp=now.to_msg(),
                            timestamp_local=now.to_msg(),
                            online=self.online,
                            discoverer_name=self.discoverername,
                            monitoruri=self.monitoruri)))
                    result = True
        if rate >= DiscoveredMaster.MIN_HZ_FOR_QUALITY:
            if self.heartbeat_rate != rate:
                self.heartbeat_rate = rate
                self.heartbeats = list()
            self.heartbeats.append(cur_time)
        return result

    def add_request(self, timestamp):
        self.ts_last_request = timestamp
        self.requests.append(timestamp)

    def requests_count(self):
        return len(self.requests)

    def remove_heartbeats(self, timestamp):
        do_remove = True
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
        if self.online:
            self.online = False
            if self.callback_master_state is not None:
                Log.info('Set host to offline: %s' % self.mastername)
                clock = Clock(clock_type=ClockType.ROS_TIME)
                now = clock.now()
                self.callback_master_state(MasterState(
                    state=MasterState.STATE_CHANGED,
                    master=ROSMaster(
                        name=str(self.mastername),
                        uri=self.masteruri,
                        timestamp=now.to_msg(),
                        timestamp_local=now.to_msg(),
                        online=False,
                        discoverer_name=self.discoverername,
                        monitoruri=self.monitoruri)))

    def get_quality(self, interval=5, offline_after=1.4):
        quality = -1.0
        if self.mastername is not None and self.heartbeat_rate >= self.MIN_HZ_FOR_QUALITY:
            current_time = time.time()
            measurement_duration = interval
            if self.heartbeat_rate < 1.:
                measurement_duration = measurement_duration / self.heartbeat_rate
            if measurement_duration > current_time - self.creation_ts:
                measurement_duration = current_time - self.creation_ts
            ts_oldest = current_time - measurement_duration
            self.remove_heartbeats(ts_oldest)
            if current_time - self.last_heartbeat_ts > (measurement_duration * offline_after):
                self.set_offline()
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
        except Exception:
            pass

    def __start_get_info_timer(self, timetosleep):
        self._get_into_timer = threading.Timer(
            timetosleep, self._get_info_threaded)
        self._get_into_timer.start()

    def _get_info_threaded(self):
        thread = threading.Thread(target=self._retrieve_masterinfo)
        thread.daemon = True
        thread.start()

    def _retrieve_masterinfo(self):
        '''
        In ROS 2, we no longer use XML-RPC to retrieve master info.
        This is a simplified placeholder that sets basic info.
        '''
        if self.monitoruri is not None and not self._on_finish:
            timetosleep = 5.
            if self.mastername is None:
                try:
                    # In ROS 2, discovery is handled differently
                    # For now, extract info from the monitor URI
                    self.master_hostname = get_hostname(self.monitoruri)
                    if self.master_hostname:
                        self.masteruri = self.monitoruri
                        self.mastername = self.master_hostname
                        self.discoverername = 'discoverer'
                        self.online = True
                        try:
                            self.masteruriaddr = socket.gethostbyname(
                                self.master_hostname)
                            self._del_error(self.ERR_RESOLVE_NAME)
                        except socket.gaierror:
                            msg = "Master discovered with not known hostname: '%s'" % str(
                                self.masteruri)
                            Log.warn(msg)
                            self._add_error(self.ERR_RESOLVE_NAME, msg)
                        else:
                            if self.callback_master_state is not None:
                                Log.info(
                                    "Added master with URI=%s" % (self.masteruri))
                                clock = Clock(clock_type=ClockType.ROS_TIME)
                                now = clock.now()
                                self.callback_master_state(MasterState(
                                    state=MasterState.STATE_NEW,
                                    master=ROSMaster(
                                        name=str(self.mastername),
                                        uri=self.masteruri,
                                        timestamp=now.to_msg(),
                                        timestamp_local=now.to_msg(),
                                        online=self.online,
                                        discoverer_name=self.discoverername,
                                        monitoruri=self.monitoruri)))
                                timetosleep = 0
                except Exception:
                    msg = "can't retrieve connection information from [%s]: %s" % (
                        self.monitoruri, traceback.format_exc())
                    Log.warn(msg)
                    self._add_error(self.ERR_SOCKET, msg)

                if not self._on_finish and timetosleep > 0:
                    self.__start_get_info_timer(timetosleep)


class Discoverer(object):
    '''
    The class to publish the current state of the ROS master.

    Discovering is done by heartbeats:
      Each master discovery node sends to a multicast group periodically messages
      with current state.
    '''

    VERSION = 2
    HEARTBEAT_FMT = 'cBBiiHii'
    HEARTBEAT_HZ = 0.02
    MEASUREMENT_INTERVALS = 5
    TIMEOUT_FACTOR = 1
    ROSMASTER_HZ = 1
    REMOVE_AFTER = 300
    ACTIVE_REQUEST_AFTER = 60
    INIT_NOTIFICATION_COUNT = 3
    OFFLINE_AFTER_REQUEST_COUNT = 5
    CHANGE_NOTIFICATION_COUNT = 3
    NETPACKET_SIZE = 68

    def __init__(self, mcast_port, mcast_group, monitor_port, ros2_node=None):
        '''
        Initialize method for the Discoverer class

        :param mcast_port: The port used to publish and receive the multicast messages.
        :type mcast_port:  int
        :param mcast_group: The IPv4 or IPv6 multicast group used for discovering over nodes.
        :type mcast_group:  str
        :param monitor_port: The port of the RPC Server, used to get more information about the ROS master.
        :type monitor_port:  int
        :param ros2_node: An rclpy Node instance for creating publishers
        :type ros2_node: rclpy.node.Node
        '''
        self.do_finish = False
        self._services_initialized = False
        self.__lock = threading.RLock()
        self.masters = dict()
        self._changed = False
        self._last_datetime = time.time()

        # Store the ROS 2 node reference
        self._node = ros2_node

        # Initialize ROS 2 publishers for '~/changes' (MasterState)
        # and '~/linkstats' (LinkStatesStamped) using the node's create_publisher method.
        if self._node is not None:
            self.pubchanges = self._node.create_publisher(MasterState, '~/changes', 10)
            self.pubstats = self._node.create_publisher(LinkStatesStamped, '~/linkstats', 1)
        else:
            self.pubchanges = None
            self.pubstats = None

        self.mcast_port = mcast_port
        self.mcast_group = mcast_group
        self.monitor_port = monitor_port
        self._current_change_notification_count = 0
        self._init_notifications = 0

        if self.HEARTBEAT_HZ <= 0.:
            Log.warn(
                "Heart beat [Hz]: %s is increased to 0.02" % self.HEARTBEAT_HZ)
            self.HEARTBEAT_HZ = 0.02
        if self.HEARTBEAT_HZ > 25.5:
            Log.warn(
                "Heart beat [Hz]: %s is decreased to 25.5" % self.HEARTBEAT_HZ)
            self.HEARTBEAT_HZ = 25.5

        if self.HEARTBEAT_HZ > DiscoveredMaster.MIN_HZ_FOR_QUALITY:
            self._current_change_notification_count = self.CHANGE_NOTIFICATION_COUNT

    def _publish_current_state(self, address=None, msg=None):
        '''
        Handle logic for sending heartbeat messages.
        If 'msg' is None, generate a new heartbeat packet using 'struct.pack'
        with the current ROS 2 timestamp (secs/nsecs) and RPC port.
        Support sending to a specific 'address' (unicast) or to the multicast group.
        '''
        if msg is None:
            # Generate heartbeat message using struct.pack with HEARTBEAT_FMT
            clock = Clock(clock_type=ClockType.ROS_TIME)
            now = clock.now()
            # Convert to seconds and nanoseconds
            nanoseconds = now.nanoseconds
            secs = int(nanoseconds // 1000000000)
            nsecs = int(nanoseconds % 1000000000)
            msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                              int(self.HEARTBEAT_HZ * 10),
                              secs, nsecs,
                              self.monitor_port,
                              secs, nsecs)

        # Send the message via socket if available
        if hasattr(self, 'socket') and self.socket is not None:
            try:
                if address is not None:
                    # Unicast to specific address
                    self.socket.send_queued(msg, address)
                else:
                    # Multicast
                    self.socket.send_queued(msg)
            except Exception as e:
                Log.warn("Error sending heartbeat: %s" % str(e))

    def _create_current_state_msg(self):
        clock = Clock(clock_type=ClockType.ROS_TIME)
        now = clock.now()
        nanoseconds = now.nanoseconds
        t = nanoseconds / 1e9
        local_t = t
        return struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                           int(self.HEARTBEAT_HZ * 10),
                           int(t), int((t - int(t)) * 1000000000),
                           self.monitor_port,
                           int(local_t), int((local_t - int(local_t)) * 1000000000))

    def _create_request_update_msg(self):
        version = Discoverer.VERSION if Discoverer.VERSION > 2 else 3
        msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', version,
                          int(self.HEARTBEAT_HZ * 10), 0, 0,
                          self.monitor_port, 0, 0)
        return msg

    @classmethod
    def msg2masterState(cls, msg, address):
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
                raise Exception("newer heartbeat version %s (own: %s) from %s detected" % (
                    version, Discoverer.VERSION, address))
            elif (version < Discoverer.VERSION):
                raise Exception("old heartbeat version %s detected (current: %s) from %s" % (
                    version, Discoverer.VERSION, address))
            else:
                raise Exception("heartbeat version %s expected, received: %s" % (
                    Discoverer.VERSION, version))
        raise Exception("message is too small")

    def publish_masterstate(self, master_state):
        '''
        Publishes the given state to the ROS network. This method is thread safe.
        '''
        with self.__lock:
            try:
                if self.pubchanges is not None:
                    self.pubchanges.publish(master_state)
            except Exception:
                traceback.print_exc()

    def publish_stats(self, stats):
        '''
        Publishes the link quality states to the ROS network. This method is thread safe.
        '''
        with self.__lock:
            try:
                if self.pubstats is not None:
                    self.pubstats.publish(stats)
            except Exception:
                traceback.print_exc()

    def on_shutdown(self, *arg):
        with self.__lock:
            self.do_finish = True
            for (_, master) in self.masters.items():
                if master.mastername is not None:
                    clock = Clock(clock_type=ClockType.ROS_TIME)
                    now = clock.now()
                    self.publish_masterstate(MasterState(
                        state=MasterState.STATE_REMOVED,
                        master=ROSMaster(
                            name=str(master.mastername),
                            uri=master.masteruri,
                            timestamp=now.to_msg(),
                            timestamp_local=now.to_msg(),
                            online=master.online,
                            discoverer_name=master.discoverername,
                            monitoruri=master.monitoruri)))
                master.finish()
            # send notification that the master is going off
            msg = struct.pack(Discoverer.HEARTBEAT_FMT, b'R', Discoverer.VERSION,
                              int(self.HEARTBEAT_HZ * 10), -1, -1,
                              self.monitor_port, -1, -1)
            self._publish_current_state(msg=msg)
            self.masters.clear()