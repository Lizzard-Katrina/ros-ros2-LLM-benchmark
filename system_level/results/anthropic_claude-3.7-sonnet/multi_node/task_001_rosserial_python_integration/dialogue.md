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

FILE_PATH: SerialClient.py
----------------------------
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
import imp
import io
import multiprocessing
import queue
import socket
import struct
import sys
import threading
import time

from serial import Serial, SerialException, SerialTimeoutException

import roslib
import rospy
from std_msgs.msg import Time
from rosserial_msgs.msg import TopicInfo, Log
from rosserial_msgs.srv import RequestParamRequest, RequestParamResponse

import diagnostic_msgs.msg

ERROR_MISMATCHED_PROTOCOL = "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client"
ERROR_NO_SYNC = "no sync with device"
ERROR_PACKET_FAILED = "Packet Failed : Failed to read msg data"

def load_pkg_module(package, directory):
    #check if its in the python path
    path = sys.path
    try:
        imp.find_module(package)
    except ImportError:
        roslib.load_manifest(package)
    try:
        m = __import__( package + '.' + directory )
    except ImportError:
        rospy.logerr( "Cannot import package : %s"% package )
        rospy.logerr( "sys.path was " + str(path) )
        return None
    return m

def load_message(package, message):
    m = load_pkg_module(package, 'msg')
    m2 = getattr(m, 'msg')
    return getattr(m2, message)

def load_service(package,service):
    s = load_pkg_module(package, 'srv')
    s = getattr(s, 'srv')
    srv = getattr(s, service)
    mreq = getattr(s, service+"Request")
    mres = getattr(s, service+"Response")
    return srv,mreq,mres

class Publisher:
    """
        Publisher forwards messages from the serial device to ROS.
    """
    def __init__(self, topic_info):
        """ Create a new publisher. """
        self.topic = topic_info.topic_name

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message._md5sum == topic_info.md5sum:
            self.publisher = rospy.Publisher(self.topic, self.message, queue_size=10)
        else:
            raise Exception('Checksum does not match: ' + self.message._md5sum + ',' + topic_info.md5sum)

    def handlePacket(self, data):
        """ Forward message to ROS network. """
        m = self.message()
        m.deserialize(data)
        self.publisher.publish(m)


class Subscriber:
    """
        Subscriber forwards messages from ROS to the serial device.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.parent = parent

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message._md5sum == topic_info.md5sum:
            self.subscriber = rospy.Subscriber(self.topic, self.message, self.callback)
        else:
            raise Exception('Checksum does not match: ' + self.message._md5sum + ',' + topic_info.md5sum)

    def callback(self, msg):
        """ Forward message to serial device. """
        data_buffer = io.BytesIO()
        msg.serialize(data_buffer)
        self.parent.send(self.id, data_buffer.getvalue())

    def unregister(self):
        rospy.loginfo("Removing subscriber: %s", self.topic)
        self.subscriber.unregister()

class ServiceServer:
    """
        ServiceServer responds to requests from ROS.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.parent = parent

        # find message type
        package, service = topic_info.message_type.split('/')
        s = load_pkg_module(package, 'srv')
        s = getattr(s, 'srv')
        self.mreq = getattr(s, service+"Request")
        self.mres = getattr(s, service+"Response")
        srv = getattr(s, service)
        self.service = rospy.Service(self.topic, srv, self.callback)

        # response message
        self.data = None

    def unregister(self):
        rospy.loginfo("Removing service: %s", self.topic)
        self.service.shutdown()

    def callback(self, req):
        """ Forward request to serial device. """
        data_buffer = io.BytesIO()
        req.serialize(data_buffer)
        self.response = None
        self.parent.send(self.id, data_buffer.getvalue())
        while self.response is None:
            pass
        return self.response

    def handlePacket(self, data):
        """ Forward response to ROS network. """
        r = self.mres()
        r.deserialize(data)
        self.response = r


class ServiceClient:
    """
        ServiceServer responds to requests from ROS.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.parent = parent

        # find message type
        package, service = topic_info.message_type.split('/')
        s = load_pkg_module(package, 'srv')
        s = getattr(s, 'srv')
        self.mreq = getattr(s, service+"Request")
        self.mres = getattr(s, service+"Response")
        srv = getattr(s, service)
        rospy.loginfo("Starting service client, waiting for service '" + self.topic + "'")
        rospy.wait_for_service(self.topic)
        self.proxy = rospy.ServiceProxy(self.topic, srv)

    def handlePacket(self, data):
        """ Forward request to ROS network. """
        req = self.mreq()
        req.deserialize(data)
        # call service proxy
        resp = self.proxy(req)
        # serialize and publish
        data_buffer = io.BytesIO()
        resp.serialize(data_buffer)
        self.parent.send(self.id, data_buffer.getvalue())

class RosSerialServer:
    """
        RosSerialServer waits for a socket connection then passes itself, forked as a
        new process, to SerialClient which uses it as a serial port. It continues to listen
        for additional connections. Each forked process is a new ros node, and proxies ros
        operations (e.g. publish/subscribe) from its connection to the rest of ros.
    """
    def __init__(self, tcp_portnum, fork_server=False):
        rospy.loginfo("Fork_server is: %s" % fork_server)
        self.tcp_portnum = tcp_portnum
        self.fork_server = fork_server

    def listen(self):
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #bind the socket to a public host, and a well-known port
        self.serversocket.bind(("", self.tcp_portnum)) #become a server socket
        self.serversocket.listen(1)
        self.serversocket.settimeout(1)

        #accept connections
        rospy.loginfo("Waiting for socket connection")
        while not rospy.is_shutdown():
            try:
                clientsocket, address = self.serversocket.accept()
            except socket.timeout:
                continue

            #now do something with the clientsocket
            rospy.loginfo("Established a socket connection from %s on port %s" % address)
            self.socket = clientsocket
            self.isConnected = True

            if self.fork_server: # if configured to launch server in a separate process
                rospy.loginfo("Forking a socket server process")
                process = multiprocessing.Process(target=self.startSocketServer, args=address)
                process.daemon = True
                process.start()
                rospy.loginfo("launched startSocketServer")
            else:
                rospy.loginfo("calling startSerialClient")
                self.startSerialClient()
                rospy.loginfo("startSerialClient() exited")

    def startSerialClient(self):
        client = SerialClient(self)
        try:
            client.run()
        except KeyboardInterrupt:
            pass
        except RuntimeError:
            rospy.loginfo("RuntimeError exception caught")
            self.isConnected = False
        except socket.error:
            rospy.loginfo("socket.error exception caught")
            self.isConnected = False
        finally:
            rospy.loginfo("Client has exited, closing socket.")
            self.socket.close()
            for sub in client.subscribers.values():
                sub.unregister()
            for srv in client.services.values():
                srv.unregister()

    def startSocketServer(self, port, address):
        rospy.loginfo("starting ROS Serial Python Node serial_node-%r" % address)
        rospy.init_node("serial_node_%r" % address)
        self.startSerialClient()

    def flushInput(self):
        pass

    def write(self, data):
        if not self.isConnected:
            return
        length = len(data)
        totalsent = 0

        while totalsent < length:
            try:
                totalsent += self.socket.send(data[totalsent:])
            except BrokenPipeError:
                raise RuntimeError("RosSerialServer.write() socket connection broken")

    def read(self, rqsted_length):
        self.msg = b''
        if not self.isConnected:
            return self.msg

        while len(self.msg) < rqsted_length:
            chunk = self.socket.recv(rqsted_length - len(self.msg))
            if chunk == b'':
                raise RuntimeError("RosSerialServer.read() socket connection broken")
            self.msg = self.msg + chunk
        return self.msg

    def inWaiting(self):
        try: # the caller checks just for <1, so we'll peek at just one byte
            chunk = self.socket.recv(1, socket.MSG_DONTWAIT|socket.MSG_PEEK)
            if chunk == b'':
                raise RuntimeError("RosSerialServer.inWaiting() socket connection broken")
            return len(chunk)
        except BlockingIOError:
            return 0

class SerialClient(object):
    """
        ServiceServer responds to requests from the serial device.
    """
    header = b'\xff'

    # hydro introduces protocol ver2 which must match node_handle.h
    # The protocol version is sent as the 2nd sync byte emitted by each end
    protocol_ver1 = b'\xff'
    protocol_ver2 = b'\xfe'
    protocol_ver = protocol_ver2

    def __init__(self, port=None, baud=57600, timeout=5.0, fix_pyserial_for_test=False):
        """ Initialize node, connect to bus, attempt to negotiate topics. """

        #TODO
        # 1. Dependency Alignment: Ensure the client is initialized to use an externally 
#    provided ROS 2 node instance for all communications (Publishing, Subscribing, Logging).
# 2. State & Timing: Synchronize internal status tracking and diagnostic tools 
#    with the ROS 2 time system provided by the injected node context.
# 3. Resource Mapping: Properly initialize all communication objects (e.g., diagnostics) 
#    via the node-based API rather than global ROS 1 patterns.
# 
# Constraints:
# - All ROS 2 interface creations (publishers, etc.) must use 'node.create_...'.
# - Forbid 'rclpy.create_node' inside this class to maintain decoupled architecture.
        # End o TODO        
        def shutdown():
            self.txStopRequest()
            rospy.loginfo('shutdown hook activated')
        rospy.on_shutdown(shutdown)
        
        self.pub_diagnostics = rospy.Publisher('/diagnostics', diagnostic_msgs.msg.DiagnosticArray, queue_size=10)

        if port is None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
            #assume its a filelike object
            self.port=port
        else:
            # open a specific port
            while not rospy.is_shutdown():
                try:
                    if self.fix_pyserial_for_test:
                        # see https://github.com/pyserial/pyserial/issues/59
                        self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10, rtscts=True, dsrdtr=True)
                    else:
                        self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10)
                    break
                except SerialException as e:
                    rospy.logerr("Error opening serial: %s", e)
                    time.sleep(3)

        if rospy.is_shutdown():
            return

        time.sleep(0.1)           # Wait for ready (patch for Uno)

        self.buffer_out = -1
        self.buffer_in = -1

        self.callbacks = dict()
        # endpoints for creating new pubs/subs
        self.callbacks[TopicInfo.ID_PUBLISHER] = self.setupPublisher
        self.callbacks[TopicInfo.ID_SUBSCRIBER] = self.setupSubscriber
        # service client/servers have 2 creation endpoints (a publisher and a subscriber)
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        # custom endpoints
        self.callbacks[TopicInfo.ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[TopicInfo.ID_LOG] = self.handleLoggingRequest
        self.callbacks[TopicInfo.ID_TIME] = self.handleTimeRequest

        rospy.sleep(2.0)
        self.requestTopics()
        self.lastsync = rospy.Time.now()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        rospy.loginfo('Requesting topics...')

        if not self.fix_pyserial_for_test:
            with self.read_lock:
                self.port.flushInput()

        # request topic sync
        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x00\x00\xff")

    def txStopRequest(self):
        """ Send stop tx request to client before the node exits. """
        if not self.fix_pyserial_for_test:
            with self.read_lock:
                self.port.flushInput()

        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x0b\x00\xf4")
        rospy.loginfo("Sending tx stop request")

    def tryRead(self, length):
        try:
            read_start = time.time()
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    self.last_read = rospy.Time.now()
                    result.extend(received)
                    bytes_remaining -= len(received)

            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)

    def run(self):
        """ Forward recieved messages to appropriate publisher. """

        # Launch write thread.
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self.processWriteQueue)
            self.write_thread.daemon = True
            self.write_thread.start()

        # Handle reading.
        data = ''
        read_step = None
        while self.write_thread.is_alive() and not rospy.is_shutdown():
            if (rospy.Time.now() - self.lastsync).to_sec() > (self.timeout * 3):
                if self.synced:
                    rospy.logerr("Lost sync with device, restarting...")
                else:
                    rospy.logerr("Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino")
                self.lastsync_lost = rospy.Time.now()
                self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_NO_SYNC)
                self.requestTopics()
                self.lastsync = rospy.Time.now()

            # This try-block is here because we make multiple calls to read(). Any one of them can throw
            # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
            # bottom attempts to reconfigure the topics.
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
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_MISMATCHED_PROTOCOL)
                    rospy.logerr("Mismatched protocol version in packet (%s): lost sync or rosserial_python is from different ros release than the rosserial client" % repr(flag[1]))
                    protocol_ver_msgs = {
                            self.protocol_ver1: 'Rev 0 (rosserial 0.4 and earlier)',
                            self.protocol_ver2: 'Rev 1 (rosserial 0.5+)',
                            b'\xfd': 'Some future rosserial version'
                    }
                    if flag[1] in protocol_ver_msgs:
                        found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                    else:
                        found_ver_msg = "Protocol version of client is unrecognized"
                    rospy.loginfo("%s, expected %s" % (found_ver_msg, protocol_ver_msgs[self.protocol_ver]))
                    continue

                # Read message length, checksum (3 bytes)
                read_step = 'message length'
                msg_len_bytes = self.tryRead(3)
                msg_length, _ = struct.unpack("<hB", msg_len_bytes)

                # Validate message length checksum.
                if sum(array.array("B", msg_len_bytes)) % 256 != 255:
                    rospy.loginfo("Wrong checksum for msg length, length %d, dropping message." % (msg_length))
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
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_PACKET_FAILED)
                    rospy.loginfo("Packet Failed :  Failed to read msg data")
                    rospy.loginfo("expected msg length is %d", msg_length)
                    raise

                # Reada checksum for topic id and msg
                read_step = 'data checksum'
                chk = self.tryRead(1)
                checksum = sum(array.array('B', topic_id_header + msg + chk))

                # Validate checksum.
                if checksum % 256 == 255:
                    self.synced = True
                    self.lastsync_success = rospy.Time.now()
                    try:
                        self.callbacks[topic_id](msg)
                    except KeyError:
                        rospy.logerr("Tried to publish before configured, topic id %d" % topic_id)
                        self.requestTopics()
                    time.sleep(0.001)
                else:
                    rospy.loginfo("wrong checksum for topic id and msg")

            except IOError as exc:
                rospy.logwarn('Last read step: %s' % read_step)
                rospy.logwarn('Run loop error: %s' % exc)
                # One of the read calls had an issue. Just to be safe, request that the client
                # reinitialize their topics.
                with self.read_lock:
                    self.port.flushInput()
                with self.write_lock:
                    self.port.flushOutput()
                self.requestTopics()
        self.write_thread.join()

    def setPublishSize(self, size):
        if self.buffer_out < 0:
            self.buffer_out = size
            rospy.loginfo("Note: publish buffer size is %d bytes" % self.buffer_out)

    def setSubscribeSize(self, size):
        if self.buffer_in < 0:
            self.buffer_in = size
            rospy.loginfo("Note: subscribe buffer size is %d bytes" % self.buffer_in)

    def setupPublisher(self, data):
        """ Register a new publisher. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            pub = Publisher(msg)
            self.publishers[msg.topic_id] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
            rospy.loginfo("Setup publisher on %s [%s]" % (msg.topic_name, msg.message_type) )
        except Exception as e:
            rospy.logerr("Creation of publisher failed: %s", e)

    def setupSubscriber(self, data):
        """ Register a new subscriber. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if not msg.topic_name in list(self.subscribers.keys()):
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                rospy.loginfo("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type) )
            elif msg.message_type != self.subscribers[msg.topic_name].message._type:
                old_message_type = self.subscribers[msg.topic_name].message._type
                self.subscribers[msg.topic_name].unregister()
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                rospy.loginfo("Change the message type of subscriber on %s from [%s] to [%s]" % (msg.topic_name, old_message_type, msg.message_type) )
        except Exception as e:
            rospy.logerr("Creation of subscriber failed: %s", e)

    def setupServiceServerPublisher(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(msg, self)
                rospy.loginfo("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mres._md5sum == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mres._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rospy.logerr("Creation of service server failed: %s", e)

    def setupServiceServerSubscriber(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(msg, self)
                rospy.loginfo("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mreq._md5sum == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rospy.logerr("Creation of service server failed: %s", e)

    def setupServiceClientPublisher(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(msg, self)
                rospy.loginfo("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mreq._md5sum == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rospy.logerr("Creation of service client failed: %s", e)

    def setupServiceClientSubscriber(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(msg, self)
                rospy.loginfo("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mres._md5sum == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mres._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rospy.logerr("Creation of service client failed: %s", e)

    def handleTimeRequest(self, data):
        """ Respond to device with system time. """
        t = Time()
        t.data = rospy.Time.now()
        data_buffer = io.BytesIO()
        t.serialize(data_buffer)
        self.send( TopicInfo.ID_TIME, data_buffer.getvalue() )
        self.lastsync = rospy.Time.now()

    def handleParameterRequest(self, data):
        """ Send parameters to device. Supports only simple datatypes and arrays of such. """
        req = RequestParamRequest()
        req.deserialize(data)
        resp = RequestParamResponse()
        try:
            param = rospy.get_param(req.name)
        except KeyError:
            rospy.logerr("Parameter %s does not exist"%req.name)
            return

        if param is None:
            rospy.logerr("Parameter %s does not exist"%req.name)
            return

        if isinstance(param, dict):
            rospy.logerr("Cannot send param %s because it is a dictionary"%req.name)
            return
        if not isinstance(param, list):
            param = [param]
        #check to make sure that all parameters in list are same type
        t = type(param[0])
        for p in param:
            if t!= type(p):
                rospy.logerr('All Paramers in the list %s must be of the same type'%req.name)
                return
        if t == int or t == bool:
            resp.ints = param
        if t == float:
            resp.floats =param
        if t == str:
            resp.strings = param
        data_buffer = io.BytesIO()
        resp.serialize(data_buffer)
        self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getvalue())

    def handleLoggingRequest(self, data):
        """ Forward logging information from serial device into ROS. """
        msg = Log()
        msg.deserialize(data)
        if msg.level == Log.ROSDEBUG:
            rospy.logdebug(msg.msg)
        elif msg.level == Log.INFO:
            rospy.loginfo(msg.msg)
        elif msg.level == Log.WARN:
            rospy.logwarn(msg.msg)
        elif msg.level == Log.ERROR:
            rospy.logerr(msg.msg)
        elif msg.level == Log.FATAL:
            rospy.logfatal(msg.msg)

    def send(self, topic, msg):
        """
        Queues data to be written to the serial port.
        """
        self.write_queue.put((topic, msg))

    def _write(self, data):
        """
        Writes raw data over the serial port. Assumes the data is formatting as a packet. http://wiki.ros.org/rosserial/Overview/Protocol
        """
        with self.write_lock:
            self.port.write(data)
            self.last_write = rospy.Time.now()

    def _send(self, topic, msg_bytes):
        """
        Send a message on a particular topic to the device.
        """
        length = len(msg_bytes)
        if self.buffer_in > 0 and length > self.buffer_in:
            rospy.logerr("Message from ROS network dropped: message larger than buffer.\n%s" % msg)
            return -1
        else:
            # frame : header (1b) + version (1b) + msg_len(2b) + msg_len_chk(1b) + topic_id(2b) + msg(nb) + msg_topic_id_chk(1b)
            length_bytes = struct.pack('<h', length)
            length_checksum = 255 - (sum(array.array('B', length_bytes)) % 256)
            length_checksum_bytes = struct.pack('B', length_checksum)

            topic_bytes = struct.pack('<h', topic)
            msg_checksum = 255 - (sum(array.array('B', topic_bytes + msg_bytes)) % 256)
            msg_checksum_bytes = struct.pack('B', msg_checksum)

            self._write(self.header + self.protocol_ver + length_bytes + length_checksum_bytes + topic_bytes + msg_bytes + msg_checksum_bytes)
            return length

    def processWriteQueue(self):
        """
        Main loop for the thread that processes outgoing data to write to the serial port.
        """
        while not rospy.is_shutdown():
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
                            rospy.logerr("Trying to write invalid data type: %s" % type(data))
                        break
                    except SerialTimeoutException as exc:
                        rospy.logerr('Write timeout: %s' % exc)
                        time.sleep(1)
                    except RuntimeError as exc:
                        rospy.logerr('Write thread exception: %s' % exc)
                        break


    def sendDiagnostics(self, level, msg_text):
        msg = diagnostic_msgs.msg.DiagnosticArray()
        status = diagnostic_msgs.msg.DiagnosticStatus()
        status.name = "rosserial_python"
        msg.header.stamp = rospy.Time.now()
        msg.status.append(status)

        status.message = msg_text
        status.level = level

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[0].key="last sync"
        if self.lastsync.to_sec()>0:
            status.values[0].value=time.ctime(self.lastsync.to_sec())
        else:
            status.values[0].value="never"

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[1].key="last sync lost"
        status.values[1].value=time.ctime(self.lastsync_lost.to_sec())

        self.pub_diagnostics.publish(msg)

----------------------------

FILE_PATH: serial_node.py
----------------------------
__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rospy
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing

import sys

if __name__=="__main__":
#TODO 
# 1. Configuration Management: Implement a class that handles the declaration and 
#    retrieval of serial connection parameters (port and baud).
# 2. Component Integration: Instantiate the SerialClient in a way that allows it 
#    to utilize the node's communication resources for its internal operations.
# 3. Lifecycle Control: Establish a standard ROS 2 execution pattern (Spin) that 
#    ensures the node remains responsive and cleans up resources upon termination.
# 
# Constraints:
# - Strictly use 'self.get_parameter(...).value' for regex matching.
# - Ensure 'SerialClient' is called with 'self' as an argument.
# END OF TODO

----------------------------


# LLM Response

[FILENAME: SerialClient.py]
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

from serial import Serial, SerialException, SerialTimeoutException

import rclpy
from rclpy.node import Node
from rclpy.time import Time as rclpy_time
from rclpy.duration import Duration
from std_msgs.msg import Time
from rosserial_msgs.msg import TopicInfo, Log
from rosserial_msgs.srv import RequestParam

import diagnostic_msgs.msg

ERROR_MISMATCHED_PROTOCOL = "Mismatched protocol version in packet: lost sync or rosserial_python is from different ros release than the rosserial client"
ERROR_NO_SYNC = "no sync with device"
ERROR_PACKET_FAILED = "Packet Failed : Failed to read msg data"

def load_pkg_module(package, directory):
    try:
        m = importlib.import_module(package + '.' + directory)
    except ImportError:
        return None
    return m

def load_message(package, message):
    m = load_pkg_module(package, 'msg')
    if m is None:
        return None
    return getattr(m, message)

def load_service(package, service):
    s = load_pkg_module(package, 'srv')
    if s is None:
        return None, None, None
    srv = getattr(s, service)
    mreq = getattr(s, service + "_Request")
    mres = getattr(s, service + "_Response")
    return srv, mreq, mres

class Publisher:
    """
        Publisher forwards messages from the serial device to ROS.
    """
    def __init__(self, topic_info, node):
        """ Create a new publisher. """
        self.topic = topic_info.topic_name
        self.node = node

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message._TYPE_SUPPORT.typesupport_identifier == topic_info.md5sum:
            self.publisher = node.create_publisher(self.message, self.topic, 10)
        else:
            raise Exception('Checksum does not match: ' + self.message._TYPE_SUPPORT.typesupport_identifier + ',' + topic_info.md5sum)

    def handlePacket(self, data):
        """ Forward message to ROS network. """
        m = self.message()
        m.deserialize(data)
        self.publisher.publish(m)


class Subscriber:
    """
        Subscriber forwards messages from ROS to the serial device.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.id = topic_info.topic_id
        self.parent = parent
        self.node = parent.node

        # find message type
        package, message = topic_info.message_type.split('/')
        self.message = load_message(package, message)
        if self.message._TYPE_SUPPORT.typesupport_identifier == topic_info.md5sum:
            self.subscriber = self.node.create_subscription(
                self.message, 
                self.topic, 
                self.callback,
                10)
        else:
            raise Exception('Checksum does not match: ' + self.message._TYPE_SUPPORT.typesupport_identifier + ',' + topic_info.md5sum)

    def callback(self, msg):
        """ Forward message to serial device. """
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

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.parent = parent
        self.node = parent.node

        # find message type
        package, service = topic_info.message_type.split('/')
        srv, self.mreq, self.mres = load_service(package, service)
        self.service = self.node.create_service(srv, self.topic, self.callback)

        # response message
        self.data = None

    def unregister(self):
        self.node.get_logger().info("Removing service: %s" % self.topic)
        self.node.destroy_service(self.service)

    def callback(self, req, res):
        """ Forward request to serial device. """
        data_buffer = io.BytesIO()
        req.serialize(data_buffer)
        self.response = None
        self.parent.send(self.id, data_buffer.getvalue())
        while self.response is None:
            pass
        return self.response

    def handlePacket(self, data):
        """ Forward response to ROS network. """
        r = self.mres()
        r.deserialize(data)
        self.response = r


class ServiceClient:
    """
        ServiceServer responds to requests from ROS.
    """

    def __init__(self, topic_info, parent):
        self.topic = topic_info.topic_name
        self.parent = parent
        self.node = parent.node

        # find message type
        package, service = topic_info.message_type.split('/')
        srv, self.mreq, self.mres = load_service(package, service)
        self.node.get_logger().info("Starting service client, waiting for service '%s'" % self.topic)
        self.client = self.node.create_client(srv, self.topic)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Service not available, waiting...")
        self.node.get_logger().info("Service found")

    def handlePacket(self, data):
        """ Forward request to ROS network. """
        req = self.mreq()
        req.deserialize(data)
        # call service client
        future = self.client.call_async(req)
        
        # Wait for response
        while not future.done():
            time.sleep(0.01)
        
        resp = future.result()
        # serialize and publish
        data_buffer = io.BytesIO()
        resp.serialize(data_buffer)
        self.parent.send(self.id, data_buffer.getvalue())

class RosSerialServer:
    """
        RosSerialServer waits for a socket connection then passes itself, forked as a
        new process, to SerialClient which uses it as a serial port. It continues to listen
        for additional connections. Each forked process is a new ros node, and proxies ros
        operations (e.g. publish/subscribe) from its connection to the rest of ros.
    """
    def __init__(self, node, tcp_portnum, fork_server=False):
        self.node = node
        self.tcp_portnum = tcp_portnum
        self.fork_server = fork_server
        self.node.get_logger().info("Fork_server is: %s" % fork_server)

    def listen(self):
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #bind the socket to a public host, and a well-known port
        self.serversocket.bind(("", self.tcp_portnum)) #become a server socket
        self.serversocket.listen(1)
        self.serversocket.settimeout(1)

        #accept connections
        self.node.get_logger().info("Waiting for socket connection")
        while rclpy.ok():
            try:
                clientsocket, address = self.serversocket.accept()
            except socket.timeout:
                continue

            #now do something with the clientsocket
            self.node.get_logger().info("Established a socket connection from %s on port %s" % address)
            self.socket = clientsocket
            self.isConnected = True

            if self.fork_server: # if configured to launch server in a separate process
                self.node.get_logger().info("Forking a socket server process")
                process = multiprocessing.Process(target=self.startSocketServer, args=address)
                process.daemon = True
                process.start()
                self.node.get_logger().info("launched startSocketServer")
            else:
                self.node.get_logger().info("calling startSerialClient")
                self.startSerialClient()
                self.node.get_logger().info("startSerialClient() exited")

    def startSerialClient(self):
        client = SerialClient(self.node, self)
        try:
            client.run()
        except KeyboardInterrupt:
            pass
        except RuntimeError:
            self.node.get_logger().info("RuntimeError exception caught")
            self.isConnected = False
        except socket.error:
            self.node.get_logger().info("socket.error exception caught")
            self.isConnected = False
        finally:
            self.node.get_logger().info("Client has exited, closing socket.")
            self.socket.close()
            for sub in client.subscribers.values():
                sub.unregister()
            for srv in client.services.values():
                srv.unregister()

    def startSocketServer(self, port, address):
        self.node.get_logger().info("starting ROS Serial Python Node serial_node-%r" % address)
        node = rclpy.create_node("serial_node_%r" % address)
        self.startSerialClient()

    def flushInput(self):
        pass

    def write(self, data):
        if not self.isConnected:
            return
        length = len(data)
        totalsent = 0

        while totalsent < length:
            try:
                totalsent += self.socket.send(data[totalsent:])
            except BrokenPipeError:
                raise RuntimeError("RosSerialServer.write() socket connection broken")

    def read(self, rqsted_length):
        self.msg = b''
        if not self.isConnected:
            return self.msg

        while len(self.msg) < rqsted_length:
            chunk = self.socket.recv(rqsted_length - len(self.msg))
            if chunk == b'':
                raise RuntimeError("RosSerialServer.read() socket connection broken")
            self.msg = self.msg + chunk
        return self.msg

    def inWaiting(self):
        try: # the caller checks just for <1, so we'll peek at just one byte
            chunk = self.socket.recv(1, socket.MSG_DONTWAIT|socket.MSG_PEEK)
            if chunk == b'':
                raise RuntimeError("RosSerialServer.inWaiting() socket connection broken")
            return len(chunk)
        except BlockingIOError:
            return 0

class SerialClient(object):
    """
        ServiceServer responds to requests from the serial device.
    """
    header = b'\xff'

    # hydro introduces protocol ver2 which must match node_handle.h
    # The protocol version is sent as the 2nd sync byte emitted by each end
    protocol_ver1 = b'\xff'
    protocol_ver2 = b'\xfe'
    protocol_ver = protocol_ver2

    def __init__(self, node, port=None, baud=57600, timeout=5.0, fix_pyserial_for_test=False):
        """ Initialize node, connect to bus, attempt to negotiate topics. """
        self.node = node
        self.read_lock = threading.RLock()
        self.write_lock = threading.RLock()
        self.write_queue = queue.Queue()
        self.write_thread = None
        
        self.timeout = timeout
        self.synced = False
        self.fix_pyserial_for_test = fix_pyserial_for_test
        
        self.lastsync = self.node.get_clock().now()
        self.lastsync_lost = self.node.get_clock().now()
        self.lastsync_success = self.node.get_clock().now()
        self.last_read = self.node.get_clock().now()
        self.last_write = self.node.get_clock().now()
        
        self.publishers = dict()
        self.subscribers = dict()
        self.services = dict()
        
        def shutdown():
            self.txStopRequest()
            self.node.get_logger().info('shutdown hook activated')
        
        self.pub_diagnostics = self.node.create_publisher(
            diagnostic_msgs.msg.DiagnosticArray, 
            '/diagnostics', 
            10)

        if port is None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
            #assume its a filelike object
            self.port=port
        else:
            # open a specific port
            while rclpy.ok():
                try:
                    if self.fix_pyserial_for_test:
                        # see https://github.com/pyserial/pyserial/issues/59
                        self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10, rtscts=True, dsrdtr=True)
                    else:
                        self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10)
                    break
                except SerialException as e:
                    self.node.get_logger().error("Error opening serial: %s" % e)
                    time.sleep(3)

        if not rclpy.ok():
            return

        time.sleep(0.1)           # Wait for ready (patch for Uno)

        self.buffer_out = -1
        self.buffer_in = -1

        self.callbacks = dict()
        # endpoints for creating new pubs/subs
        self.callbacks[TopicInfo.ID_PUBLISHER] = self.setupPublisher
        self.callbacks[TopicInfo.ID_SUBSCRIBER] = self.setupSubscriber
        # service client/servers have 2 creation endpoints (a publisher and a subscriber)
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        # custom endpoints
        self.callbacks[TopicInfo.ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[TopicInfo.ID_LOG] = self.handleLoggingRequest
        self.callbacks[TopicInfo.ID_TIME] = self.handleTimeRequest

        time.sleep(2.0)
        self.requestTopics()
        self.lastsync = self.node.get_clock().now()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        self.node.get_logger().info('Requesting topics...')

        if not self.fix_pyserial_for_test:
            with self.read_lock:
                self.port.flushInput()

        # request topic sync
        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x00\x00\xff")

    def txStopRequest(self):
        """ Send stop tx request to client before the node exits. """
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
        """ Forward recieved messages to appropriate publisher. """

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
            if (now - self.lastsync).nanoseconds / 1e9 > (self.timeout * 3):
                if self.synced:
                    self.node.get_logger().error("Lost sync with device, restarting...")
                else:
                    self.node.get_logger().error("Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino")
                self.lastsync_lost = now
                self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_NO_SYNC)
                self.requestTopics()
                self.lastsync = now

            # This try-block is here because we make multiple calls to read(). Any one of them can throw
            # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
            # bottom attempts to reconfigure the topics.
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
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_MISMATCHED_PROTOCOL)
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
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_PACKET_FAILED)
                    self.node.get_logger().info("Packet Failed :  Failed to read msg data")
                    self.node.get_logger().info("expected msg length is %d", msg_length)
                    raise

                # Reada checksum for topic id and msg
                read_step = 'data checksum'
                chk = self.tryRead(1)
                checksum = sum(array.array('B', topic_id_header + msg + chk))

                # Validate checksum.
                if checksum % 256 == 255:
                    self.synced = True
                    self.lastsync_success = now
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
                # One of the read calls had an issue. Just to be safe, request that the client
                # reinitialize their topics.
                with self.read_lock:
                    self.port.flushInput()
                with self.write_lock:
                    self.port.flushOutput()
                self.requestTopics()
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
        """ Register a new publisher. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            pub = Publisher(msg, self.node)
            self.publishers[msg.topic_id] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
            self.node.get_logger().info("Setup publisher on %s [%s]" % (msg.topic_name, msg.message_type) )
        except Exception as e:
            self.node.get_logger().error("Creation of publisher failed: %s", e)

    def setupSubscriber(self, data):
        """ Register a new subscriber. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if not msg.topic_name in list(self.subscribers.keys()):
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                self.node.get_logger().info("Setup subscriber on %s [%s]" % (msg.topic_name, msg.message_type) )
            elif msg.message_type != self.subscribers[msg.topic_name].message._type:
                old_message_type = self.subscribers[msg.topic_name].message._type
                self.subscribers[msg.topic_name].unregister()
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                self.node.get_logger().info("Change the message type of subscriber on %s from [%s] to [%s]" % (msg.topic_name, old_message_type, msg.message_type) )
        except Exception as e:
            self.node.get_logger().error("Creation of subscriber failed: %s", e)

    def setupServiceServerPublisher(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(msg, self)
                self.node.get_logger().info("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mres._TYPE_SUPPORT.typesupport_identifier == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mres._TYPE_SUPPORT.typesupport_identifier + ',' + msg.md5sum)
        except Exception as e:
            self.node.get_logger().error("Creation of service server failed: %s", e)

    def setupServiceServerSubscriber(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(msg, self)
                self.node.get_logger().info("Setup service server on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mreq._TYPE_SUPPORT.typesupport_identifier == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._TYPE_SUPPORT.typesupport_identifier + ',' + msg.md5sum)
        except Exception as e:
            self.node.get_logger().error("Creation of service server failed: %s", e)

    def setupServiceClientPublisher(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(msg, self)
                self.node.get_logger().info("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mreq._TYPE_SUPPORT.typesupport_identifier == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._TYPE_SUPPORT.typesupport_identifier + ',' + msg.md5sum)
        except Exception as e:
            self.node.get_logger().error("Creation of service client failed: %s", e)

    def setupServiceClientSubscriber(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(msg, self)
                self.node.get_logger().info("Setup service client on %s [%s]" % (msg.topic_name, msg.message_type) )
                self.services[msg.topic_name] = srv
            if srv.mres._TYPE_SUPPORT.typesupport_identifier == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mres._TYPE_SUPPORT.typesupport_identifier + ',' + msg.md5sum)
        except Exception as e:
            self.node.get_logger().error("Creation of service client failed: %s", e)

    def handleTimeRequest(self, data):
        """ Respond to device with system time. """
        t = Time()
        t.data = self.node.get_clock().now().to_msg()
        data_buffer = io.BytesIO()
        t.serialize(data_buffer)
        self.send(TopicInfo.ID_TIME, data_buffer.getvalue())
        self.lastsync = self.node.get_clock().now()

    def handleParameterRequest(self, data):
        """ Send parameters to device. Supports only simple datatypes and arrays of such. """
        req = RequestParam.Request()
        req.deserialize(data)
        resp = RequestParam.Response()
        try:
            param = self.node.get_parameter(req.name).value
        except:
            self.node.get_logger().error("Parameter %s does not exist" % req.name)
            return

        if param is None:
            self.node.get_logger().error("Parameter %s does not exist" % req.name)
            return

        if isinstance(param, dict):
            self.node.get_logger().error("Cannot send param %s because it is a dictionary" % req.name)
            return
        if not isinstance(param, list):
            param = [param]
        #check to make sure that all parameters in list are same type
        t = type(param[0])
        for p in param:
            if t != type(p):
                self.node.get_logger().error('All Parameters in the list %s must be of the same type' % req.name)
                return
        if t == int or t == bool:
            resp.ints = param
        if t == float:
            resp.floats = param
        if t == str:
            resp.strings = param
        data_buffer = io.BytesIO()
        resp.serialize(data_buffer)
        self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getvalue())

    def handleLoggingRequest(self, data):
        """ Forward logging information from serial device into ROS. """
        msg = Log()
        msg.deserialize(data)
        if msg.level == Log.ROSDEBUG:
            self.node.get_logger().debug(msg.msg)
        elif msg.level == Log.INFO:
            self.node.get_logger().info(msg.msg)
        elif msg.level == Log.WARN:
            self.node.get_logger().warn(msg.msg)
        elif msg.level == Log.ERROR:
            self.node.get_logger().error(msg.msg)
        elif msg.level == Log.FATAL:
            self.node.get_logger().fatal(msg.msg)

    def send(self, topic, msg):
        """
        Queues data to be written to the serial port.
        """
        self.write_queue.put((topic, msg))

    def _write(self, data):
        """
        Writes raw data over the serial port. Assumes the data is formatting as a packet. http://wiki.ros.org/rosserial/Overview/Protocol
        """
        with self.write_lock:
            self.port.write(data)
            self.last_write = self.node.get_clock().now()

    def _send(self, topic, msg_bytes):
        """
        Send a message on a particular topic to the device.
        """
        length = len(msg_bytes)
        if self.buffer_in > 0 and length > self.buffer_in:
            self.node.get_logger().error("Message from ROS network dropped: message larger than buffer.")
            return -1
        else:
            # frame : header (1b) + version (1b) + msg_len(2b) + msg_len_chk(1b) + topic_id(2b) + msg(nb) + msg_topic_id_chk(1b)
            length_bytes = struct.pack('<h', length)
            length_checksum = 255 - (sum(array.array('B', length_bytes)) % 256)
            length_checksum_bytes = struct.pack('B', length_checksum)

            topic_bytes = struct.pack('<h', topic)
            msg_checksum = 255 - (sum(array.array('B', topic_bytes + msg_bytes)) % 256)
            msg_checksum_bytes = struct.pack('B', msg_checksum)

            self._write(self.header + self.protocol_ver + length_bytes + length_checksum_bytes + topic_bytes + msg_bytes + msg_checksum_bytes)
            return length

    def processWriteQueue(self):
        """
        Main loop for the thread that processes outgoing data to write to the serial port.
        """
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
        msg = diagnostic_msgs.msg.DiagnosticArray()
        status = diagnostic_msgs.msg.DiagnosticStatus()
        status.name = "rosserial_python"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status.append(status)

        status.message = msg_text
        status.level = level

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[0].key="last sync"
        if self.lastsync.nanoseconds > 0:
            status.values[0].value=time.ctime(self.lastsync.nanoseconds / 1e9)
        else:
            status.values[0].value="never"

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[1].key="last sync lost"
        status.values[1].value=time.ctime(self.lastsync_lost.nanoseconds / 1e9)

        self.pub_diagnostics.publish(msg)

[FILENAME: serial_node.py]
__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rclpy
from rclpy.node import Node
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing

import sys
import re

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('tcp_port', 11411)
        self.declare_parameter('fork_server', False)
        
        # Get parameters
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        tcp_port = self.get_parameter('tcp_port').value
        fork_server = self.get_parameter('fork_server').value
        
        self.get_logger().info("ROS Serial Python Node")
        
        # Check if port is a TCP port
        if re.match(r'^\d{1,5}$', str(port)):
            self.get_logger().info("Opening socket server on port %s" % port)
            self.server = RosSerialServer(self, int(port), fork_server)
            self.server.listen()
        else:
            self.get_logger().info("Opening serial port %s at %d baud" % (port, baud))
            try:
                self.client = SerialClient(self, port, baud)
                self.client.run()
            except KeyboardInterrupt:
                self.get_logger().info("KeyboardInterrupt, shutting down")
            except SerialException as e:
                self.get_logger().error("Serial exception: %s" % e)
            except OSError as e:
                self.get_logger().error("OS error: %s" % e)

def main(args=None):
    rclpy.init(args=args)
    
    serial_node = SerialNode()
    
    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()