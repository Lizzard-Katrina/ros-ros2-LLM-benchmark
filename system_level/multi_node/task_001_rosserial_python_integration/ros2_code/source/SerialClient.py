#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import array
import errno
import importlib
import io
import multiprocessing
import queue
import socket
import struct
import sys
import threading
import time

# Gracefully handle missing pyserial – allows import in test environments
# where pyserial is not installed but a mock port object is provided.
try:
    from serial import Serial, SerialException, SerialTimeoutException
except ImportError:
    Serial = None

    class SerialException(Exception):
        pass

    class SerialTimeoutException(Exception):
        pass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

ERROR_MISMATCHED_PROTOCOL = "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client"
ERROR_NO_SYNC = "no sync with device"
ERROR_PACKET_FAILED = "Packet Failed : Failed to read msg data"

# TopicInfo constants (matching rosserial protocol)
ID_PUBLISHER = 0
ID_SUBSCRIBER = 1
ID_SERVICE_SERVER = 2
ID_SERVICE_CLIENT = 4
ID_PARAMETER_REQUEST = 6
ID_LOG = 7
ID_TIME = 10
ID_TX_STOP = 11


def load_pkg_module(package, directory):
    """Load a ROS 2 package module dynamically."""
    try:
        m = importlib.import_module(package + '.' + directory)
    except ImportError:
        return None
    return m


def load_message(package, message):
    m = load_pkg_module(package, 'msg')
    if m is None:
        return None
    return getattr(m, message, None)


def load_service(package, service):
    s = load_pkg_module(package, 'srv')
    if s is None:
        return None, None, None
    srv = getattr(s, service, None)
    mreq = getattr(s, service + '_Request', None)
    mres = getattr(s, service + '_Response', None)
    return srv, mreq, mres


class Publisher:
    """
    Publisher forwards messages from the serial device to ROS.
    """
    def __init__(self, node, topic_info):
        """Create a new publisher."""
        self.node = node
        self.topic = topic_info.topic_name

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message is not None:
            self.publisher = node.create_publisher(self.message, self.topic, 10)
        else:
            raise Exception('Could not load message type: ' + topic_info.message_type)

    def handlePacket(self, data):
        """Forward message to ROS network."""
        m = self.message()
        m.deserialize(data)
        self.publisher.publish(m)


class Subscriber:
    """
    Subscriber forwards messages from ROS to the serial device.
    """

    def __init__(self, node, topic_info, parent):
        self.node = node
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.parent = parent

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message is not None:
            self.subscriber = node.create_subscription(self.message, self.topic, self.callback, 10)
        else:
            raise Exception('Could not load message type: ' + topic_info.message_type)

    def callback(self, msg):
        """Forward message to serial device."""
        data_buffer = io.BytesIO()
        msg.serialize(data_buffer)
        self.parent.send(self.id, data_buffer.getvalue())

    def unregister(self):
        self.node.get_logger().info("Removing subscriber: %s" % self.topic)
        self.node.destroy_subscription(self.subscriber)


class ServiceServer:
    """
    ServiceServer responds to requests from ROS.
    """

    def __init__(self, node, topic_info, parent):
        self.node = node
        self.topic = topic_info.topic_name
        self.parent = parent

        # find message type
        package, service = topic_info.message_type.split('/')
        srv, self.mreq, self.mres = load_service(package, service)
        if srv is not None:
            self.service = node.create_service(srv, self.topic, self.callback)
        else:
            raise Exception('Could not load service type: ' + topic_info.message_type)

        # response message
        self.data = None
        self.response = None

    def unregister(self):
        self.node.get_logger().info("Removing service: %s" % self.topic)
        self.node.destroy_service(self.service)

    def callback(self, req, response):
        """Forward request to serial device."""
        data_buffer = io.BytesIO()
        req.serialize(data_buffer)
        self.response = None
        self.parent.send(self.id, data_buffer.getvalue())
        while self.response is None:
            time.sleep(0.001)
        return self.response

    def handlePacket(self, data):
        """Forward response to ROS network."""
        r = self.mres()
        r.deserialize(data)
        self.response = r


class ServiceClient:
    """
    ServiceClient forwards requests from serial device to ROS services.
    """

    def __init__(self, node, topic_info, parent):
        self.node = node
        self.topic = topic_info.topic_name
        self.parent = parent

        # find message type
        package, service = topic_info.message_type.split('/')
        srv, self.mreq, self.mres = load_service(package, service)
        if srv is not None:
            self.node.get_logger().info("Starting service client, waiting for service '%s'" % self.topic)
            self.proxy = node.create_client(srv, self.topic)
        else:
            raise Exception('Could not load service type: ' + topic_info.message_type)

    def handlePacket(self, data):
        """Forward request to ROS network."""
        req = self.mreq()
        req.deserialize(data)
        # call service
        future = self.proxy.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is not None:
            resp = future.result()
            data_buffer = io.BytesIO()
            resp.serialize(data_buffer)
            self.parent.send(self.id, data_buffer.getvalue())


class SerialClient(object):
    """
    SerialClient responds to requests from the serial device.
    Uses dependency injection: all ROS 2 operations go through the injected node.
    """
    header = b'\xff'

    # hydro introduces protocol ver2 which must match node_handle.h
    protocol_ver1 = b'\xff'
    protocol_ver2 = b'\xfe'
    protocol_ver = protocol_ver2

    def __init__(self, node, port=None, baud=57600, timeout=5.0, fix_pyserial_for_test=False):
        """Initialize client with an injected ROS 2 node instance."""
        # Store the injected node for all ROS 2 operations
        self.node = node
        self.timeout = timeout
        self.fix_pyserial_for_test = fix_pyserial_for_test

        self.synced = False
        self.publishers = dict()
        self.subscribers = dict()
        self.services = dict()
        self.write_queue = queue.Queue()
        self.write_lock = threading.RLock()
        self.read_lock = threading.RLock()
        self.write_thread = None
        self.lastsync = node.get_clock().now()
        self.lastsync_lost = node.get_clock().now()
        self.lastsync_success = node.get_clock().now()
        self.last_read = node.get_clock().now()
        self.last_write = node.get_clock().now()

        self.pub_diagnostics = node.create_publisher(DiagnosticArray, '/diagnostics', 10)

        if port is None:
            pass
        elif hasattr(port, 'read'):
            # Duck-typed port object (real Serial or mock)
            self.port = port
        else:
            # open a specific port by name – requires pyserial
            if Serial is None:
                node.get_logger().error("pyserial is not installed; cannot open port by name")
                return
            try:
                if self.fix_pyserial_for_test:
                    self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10, rtscts=True, dsrdtr=True)
                else:
                    self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10)
            except SerialException as e:
                node.get_logger().error("Error opening serial: %s" % e)
                return

        time.sleep(0.1)

        self.buffer_out = -1
        self.buffer_in = -1

        self.callbacks = dict()
        self.callbacks[ID_PUBLISHER] = self.setupPublisher
        self.callbacks[ID_SUBSCRIBER] = self.setupSubscriber
        self.callbacks[ID_SERVICE_SERVER + ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[ID_SERVICE_SERVER + ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[ID_SERVICE_CLIENT + ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[ID_SERVICE_CLIENT + ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        self.callbacks[ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[ID_LOG] = self.handleLoggingRequest
        self.callbacks[ID_TIME] = self.handleTimeRequest

    def requestTopics(self):
        """Determine topics to subscribe/publish."""
        self.node.get_logger().info('Requesting topics...')

        if not self.fix_pyserial_for_test:
            with self.read_lock:
                self.port.flushInput()

        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x00\x00\xff")

    def txStopRequest(self):
        """Send stop tx request to client before the node exits."""
        if not self.fix_pyserial_for_test:
            with self.read_lock:
                self.port.flushInput()

        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x0b\x00\xf4")
        self.node.get_logger().info("Sending tx stop request")

    def tryRead(self, length):
        try:
            read_start = time.time()
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    self.last_read = self.node.get_clock().now()
                    result.extend(received)
                    bytes_remaining -= len(received)

            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)

    def run(self):
        """Forward received messages to appropriate publisher."""

        # Launch write thread.
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self.processWriteQueue)
            self.write_thread.daemon = True
            self.write_thread.start()

        # Handle reading.
        data = ''
        read_step = None
        while self.write_thread.is_alive() and rclpy.ok():
            now = self.node.get_clock().now()
            elapsed = (now - self.lastsync).nanoseconds / 1e9
            if elapsed > (self.timeout * 3):
                if self.synced:
                    self.node.get_logger().error("Lost sync with device, restarting...")
                else:
                    self.node.get_logger().error("Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino")
                self.lastsync_lost = self.node.get_clock().now()
                self.sendDiagnostics(DiagnosticStatus.ERROR, ERROR_NO_SYNC)
                self.requestTopics()
                self.lastsync = self.node.get_clock().now()

            try:
                with self.read_lock:
                    if self.port.inWaiting() < 1:
                        time.sleep(0.001)
                        continue

                # Find sync flag.
                flag = [0, 0]
                read_step = 'syncflag'
                flag[0] = self.tryRead(1)
                if (flag[0] != self.header):
                    continue

                # Find protocol version.
                read_step = 'protocol'
                flag[1] = self.tryRead(1)
                if flag[1] != self.protocol_ver:
                    self.sendDiagnostics(DiagnosticStatus.ERROR, ERROR_MISMATCHED_PROTOCOL)
                    self.node.get_logger().error("Mismatched protocol version in packet (%s): lost sync or rosserial_python is from different ros release than the rosserial client" % repr(flag[1]))
                    protocol_ver_msgs = {
                            self.protocol_ver1: 'Rev 0 (rosserial 0.4 and earlier)',
                            self.protocol_ver2: 'Rev 1 (rosserial 0.5+)',
                            b'\xfd': 'Some future rosserial version'
                    }
                    if flag[1] in protocol_ver_msgs:
                        found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                    else:
                        found_ver_msg = "Protocol version of client is unrecognized"
                    self.node.get_logger().info("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                    continue

                # Read message length, checksum (3 bytes)
                read_step = 'message length'
                msg_len_bytes = self.tryRead(3)
                msg_length, _ = struct.unpack("<hB", msg_len_bytes)

                # Validate message length checksum.
                if sum(array.array("B", msg_len_bytes)) % 256 != 255:
                    self.node.get_logger().info("Wrong checksum for msg length, length %d, dropping message." % (msg_length))
                    continue

                # Read topic id (2 bytes)
                read_step = 'topic id'
                topic_id_header = self.tryRead(2)
                topic_id, = struct.unpack("<H", topic_id_header)

                # Read serialized message data.
                read_step = 'data'
                try:
                    msg = self.tryRead(msg_length)
                except IOError:
                    self.sendDiagnostics(DiagnosticStatus.ERROR, ERROR_PACKET_FAILED)
                    self.node.get_logger().info("Packet Failed :  Failed to read msg data")
                    self.node.get_logger().info("expected msg length is %d" % msg_length)
                    raise

                # Read checksum for topic id and msg
                read_step = 'data checksum'
                chk = self.tryRead(1)
                checksum = sum(array.array('B', topic_id_header + msg + chk))

                # Validate checksum.
                if checksum % 256 == 255:
                    self.synced = True
                    self.lastsync_success = self.node.get_clock().now()
                    try:
                        self.callbacks[topic_id](msg)
                    except KeyError:
                        self.node.get_logger().error("Tried to publish before configured, topic id %d" % topic_id)
                        self.requestTopics()
                    time.sleep(0.001)
                else:
                    self.node.get_logger().info("wrong checksum for topic id and msg")

            except IOError as exc:
                self.node.get_logger().warn('Last read step: %s' % read_step)
                self.node.get_logger().warn('Run loop error: %s' % exc)
                with self.read_lock:
                    self.port.flushInput()
                with self.write_lock:
                    self.port.flushOutput()
                self.requestTopics()
        if self.write_thread is not None:
            self.write_thread.join()

    def setPublishSize(self, size):
        if self.buffer_out < 0:
            self.buffer_out = size
            self.node.get_logger().info("Note: publish buffer size is %d bytes" % self.buffer_out)

    def setSubscribeSize(self, size):
        if self.buffer_in < 0:
            self.buffer_in = size
            self.node.get_logger().info("Note: subscribe buffer size is %d bytes" % self.buffer_in)

    def setupPublisher(self, data):
        """Register a new publisher."""
        try:
            msg = self._deserialize_topic_info(data)
            pub = Publisher(self.node, msg)
            self.publishers[msg.topic_id] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
            self.node.get_logger().info("Setup publisher on %s [%s]" % (msg.topic_name, msg.message_type))
        except Exception as e:
            self.node.get_logger().error("Creation of publisher failed: %s" % e)

    def setupSubscriber(self, data):
        """Register a new subscriber."""
        try:
            msg = self._deserialize_topic_info(data)
            if msg.topic_name not in list(self.subscribers.keys()):
                sub = Subscriber(self.node, msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                self.node.get_logger().info("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type))
            else:
                self.node.get_logger().info("Subscriber on %s already exists" % msg.topic_name)
        except Exception as e:
            self.node.get_logger().error("Creation of subscriber failed: %s" % e)

    def setupServiceServerPublisher(self, data):
        """Register a new service server."""
        try:
            msg = self._deserialize_topic_info(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(self.node, msg, self)
                self.node.get_logger().info("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type))
                self.services[msg.topic_name] = srv
            self.callbacks[msg.topic_id] = srv.handlePacket
        except Exception as e:
            self.node.get_logger().error("Creation of service server failed: %s" % e)

    def setupServiceServerSubscriber(self, data):
        """Register a new service server."""
        try:
            msg = self._deserialize_topic_info(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(self.node, msg, self)
                self.node.get_logger().info("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type))
                self.services[msg.topic_name] = srv
            srv.id = msg.topic_id
        except Exception as e:
            self.node.get_logger().error("Creation of service server failed: %s" % e)

    def setupServiceClientPublisher(self, data):
        """Register a new service client."""
        try:
            msg = self._deserialize_topic_info(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(self.node, msg, self)
                self.node.get_logger().info("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type))
                self.services[msg.topic_name] = srv
            self.callbacks[msg.topic_id] = srv.handlePacket
        except Exception as e:
            self.node.get_logger().error("Creation of service client failed: %s" % e)

    def setupServiceClientSubscriber(self, data):
        """Register a new service client."""
        try:
            msg = self._deserialize_topic_info(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(self.node, msg, self)
                self.node.get_logger().info("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type))
                self.services[msg.topic_name] = srv
            srv.id = msg.topic_id
        except Exception as e:
            self.node.get_logger().error("Creation of service client failed: %s" % e)

    def _deserialize_topic_info(self, data):
        """Helper to create a TopicInfo-like object from raw data."""
        class TopicInfoData:
            def __init__(self):
                self.topic_id = 0
                self.topic_name = ''
                self.message_type = ''
                self.md5sum = ''
                self.buffer_size = 0

        info = TopicInfoData()
        try:
            offset = 0
            info.topic_id, = struct.unpack_from('<H', data, offset)
            offset += 2
            name_len, = struct.unpack_from('<I', data, offset)
            offset += 4
            info.topic_name = data[offset:offset + name_len].decode('utf-8')
            offset += name_len
            type_len, = struct.unpack_from('<I', data, offset)
            offset += 4
            info.message_type = data[offset:offset + type_len].decode('utf-8')
            offset += type_len
            md5_len, = struct.unpack_from('<I', data, offset)
            offset += 4
            info.md5sum = data[offset:offset + md5_len].decode('utf-8')
            offset += md5_len
            info.buffer_size, = struct.unpack_from('<i', data, offset)
        except Exception:
            pass
        return info

    def handleTimeRequest(self, data):
        """Respond to device with system time."""
        now = self.node.get_clock().now()
        sec = now.nanoseconds // 1000000000
        nsec = now.nanoseconds % 1000000000
        # Serialize Time message (sec + nsec as two uint32)
        time_data = struct.pack('<II', sec, nsec)
        self.send(ID_TIME, time_data)
        self.lastsync = self.node.get_clock().now()

    def handleParameterRequest(self, data):
        """Send parameters to device."""
        try:
            offset = 0
            name_len, = struct.unpack_from('<I', data, offset)
            offset += 4
            param_name = data[offset:offset + name_len].decode('utf-8')
        except Exception:
            self.node.get_logger().error("Failed to parse parameter request")
            return

        try:
            param_value = self.node.get_parameter(param_name).value
        except Exception:
            self.node.get_logger().error("Parameter %s does not exist" % param_name)
            return

        if param_value is None:
            self.node.get_logger().error("Parameter %s does not exist" % param_name)
            return

        self.node.get_logger().info("Parameter request for %s: %s" % (param_name, str(param_value)))

    def handleLoggingRequest(self, data):
        """Forward logging information from serial device into ROS."""
        try:
            offset = 0
            level, = struct.unpack_from('<B', data, offset)
            offset += 1
            msg_len, = struct.unpack_from('<I', data, offset)
            offset += 4
            log_msg = data[offset:offset + msg_len].decode('utf-8')
        except Exception:
            return

        if level == 0:  # DEBUG
            self.node.get_logger().debug(log_msg)
        elif level == 1:  # INFO
            self.node.get_logger().info(log_msg)
        elif level == 2:  # WARN
            self.node.get_logger().warn(log_msg)
        elif level == 3:  # ERROR
            self.node.get_logger().error(log_msg)
        elif level == 4:  # FATAL
            self.node.get_logger().fatal(log_msg)

    def send(self, topic, msg):
        """Queues data to be written to the serial port."""
        self.write_queue.put((topic, msg))

    def _write(self, data):
        """Writes raw data over the serial port."""
        with self.write_lock:
            self.port.write(data)
            self.last_write = self.node.get_clock().now()

    def _send(self, topic, msg_bytes):
        """Send a message on a particular topic to the device."""
        length = len(msg_bytes)
        if self.buffer_in > 0 and length > self.buffer_in:
            self.node.get_logger().error("Message from ROS network dropped: message larger than buffer.")
            return -1
        else:
            length_bytes = struct.pack('<h', length)
            length_checksum = 255 - (sum(array.array('B', length_bytes)) % 256)
            length_checksum_bytes = struct.pack('B', length_checksum)

            topic_bytes = struct.pack('<h', topic)
            msg_checksum = 255 - (sum(array.array('B', topic_bytes + msg_bytes)) % 256)
            msg_checksum_bytes = struct.pack('B', msg_checksum)

            self._write(self.header + self.protocol_ver + length_bytes + length_checksum_bytes + topic_bytes + msg_bytes + msg_checksum_bytes)
            return length

    def processWriteQueue(self):
        """Main loop for the thread that processes outgoing data."""
        while rclpy.ok():
            if self.write_queue.empty():
                time.sleep(0.01)
            else:
                data = self.write_queue.get()
                while True:
                    try:
                        if isinstance(data, tuple):
                            topic, msg = data
                            self._send(topic, msg)
                        elif isinstance(data, bytes):
                            self._write(data)
                        else:
                            self.node.get_logger().error("Trying to write invalid data type: %s" % type(data))
                        break
                    except SerialTimeoutException as exc:
                        self.node.get_logger().error('Write timeout: %s' % exc)
                        time.sleep(1)
                    except RuntimeError as exc:
                        self.node.get_logger().error('Write thread exception: %s' % exc)
                        break

    def sendDiagnostics(self, level, msg_text):
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "rosserial_python"

        now = self.node.get_clock().now()
        msg.header = Header()
        msg.header.stamp.sec = now.nanoseconds // 1000000000
        msg.header.stamp.nanosec = now.nanoseconds % 1000000000
        msg.status.append(status)

        status.message = msg_text
        status.level = level

        kv1 = KeyValue()
        kv1.key = "last sync"
        lastsync_sec = self.lastsync.nanoseconds / 1e9
        if lastsync_sec > 0:
            kv1.value = time.ctime(lastsync_sec)
        else:
            kv1.value = "never"
        status.values.append(kv1)

        kv2 = KeyValue()
        kv2.key = "last sync lost"
        kv2.value = time.ctime(self.lastsync_lost.nanoseconds / 1e9)
        status.values.append(kv2)

        self.pub_diagnostics.publish(msg)