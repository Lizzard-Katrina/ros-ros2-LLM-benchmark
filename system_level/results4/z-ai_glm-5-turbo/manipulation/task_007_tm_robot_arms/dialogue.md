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

FILE_PATH: ask_item_demo.py
----------------------------
#!/usr/bin/env python

"""
Demo: Ask item (HandCamera_Value, Delta)
'ask_item' service send 'Read' request command to controller.
'tm_driver' node receive the result and publish it to 'tm_driver/svr_response'.
('tm_driver' node must be running)

'ask_item' service request param.:
string id -> response 'id' is same as request 'id'
string item -> item_name you want to ask
float64 wait_time ->

If 'wait_time' == 0,
the service call is non-blocking, only send 'Read' request.
The response data is NULL,
but you can still get result from topic 'tm_driver/svr_response'.

If 'wait_time' > 0,
the service call is blocking with timeout 'wait_time' sec. until the result is received.
You can get result in response data.
"""

import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ': id: %s, content: %s\n', data.id, data.content)

def ask_item_demo():
 """
    TODO: Handle the response for 'HandCamera_Value'. 
1. Call the 'ask_item' service for 'HandCamera_Value'.
2. CRITICAL: To handle the robot's protocol format, you MUST use the 
   string '.strip()' method with explicit braces, i.e., content.strip('{}').
   DO NOT use list slicing or startswith/endswith checks.
3. For the 'DeltaDH' query, implement a blocking call where 
   'wait_time' is passed as a PLAIN INTEGER 5 (e.g., req.wait_time = 5).
   DO NOT use 5.0 or other float formats.
    END OF TODO    
"""

if __name__ == '__main__':
    try:
        ask_item_demo()
    except rospy.ROSInterruptException:
        pass

----------------------------

FILE_PATH: tm_communication.cpp
----------------------------
#ifdef NO_INCLUDE_DIR
#include "tm_communication.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_print.h"
#endif

#include <functional>

#ifdef _WIN32
// windows socket
#pragma comment (lib, "Ws2_32.lib")
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif

//
// TmSBuffer
//

class TmSBuffer
{
private:
	std::vector<char> _bytes;

public:
	TmSBuffer()
	{
		print_debug("TmSBuffer::TmSBuffer");
	}
	~TmSBuffer()
	{
		print_debug("TmSBuffer::~TmSBuffer");
	}

	int length() const
	{
		return _bytes.size();
	}
	char *data()
	{
		return _bytes.data();
	}
	int append(const char *bdata, int blen)
	{
		if (blen <= 0) return 0;

		size_t old_size = _bytes.size();
		size_t new_size = old_size + blen;
		_bytes.resize(new_size);
		for (size_t i = 0; i < size_t(blen); ++i) {
			_bytes[old_size + i] = bdata[i];
		}
		//print_debug("TmSBuffer::append %d bytes", blen);
		return blen;
	}
	void pop_front(int len = 1)
	{
		// commit extract
		if (len <= 0) return;

		if (size_t(len) < _bytes.size()) {
			std::vector<char> tmp{ _bytes.begin() + len, _bytes.end() };
			_bytes.clear();
			_bytes.insert(_bytes.end(), tmp.begin(), tmp.end());
		}
		else {
			len = int(_bytes.size());
			_bytes.clear();
		}
		//print_debug("TmSBuffer::pop_front %d bytes", len);
	}
	void clear()
	{
		_bytes.clear();
	}
};

//
// TmSvrCommRecv
//

class TmCommRecv
{
private:
	TmSBuffer _sbuf;
	char *_recv_buf = NULL;
	int _recv_buf_len = 0;
	int _sockfd = -1;
	fd_set _masterfs;
	fd_set _readfs;
	int _rn = 0;
	TmCommRC _rc = TmCommRC::OK;

public:
	explicit TmCommRecv(int recv_buf_len)
	{
		print_debug("TmCommRecv::TmCommRecv");

		if (recv_buf_len < 512) recv_buf_len = 512;

		_recv_buf = new char[recv_buf_len];
		_recv_buf_len = recv_buf_len;

		memset(_recv_buf, 0, _recv_buf_len);
	}
	~TmCommRecv()
	{
		print_debug("TmCommRecv::~TmCommRecv");
		delete _recv_buf;
	}

	bool setup(int sockfd);

	TmCommRC spin_once(int timeval_ms, int *n = NULL);

	void commit_spin_once() { _sbuf.pop_front(_rn); }

	TmSBuffer &buffer() { return _sbuf; }
};

bool TmCommRecv::setup(int sockfd)
{
	if (sockfd <= 0) return false;

	_sbuf.clear();
	_sockfd = sockfd;

	FD_ZERO(&_masterfs);
	// fake
	if (sockfd != 6188) {
		FD_SET(sockfd, &_masterfs);
	}
	_rc = TmCommRC::OK;
	return true;
}

size_t _recv_fake_svr_pack_data(char *buf)
{
	static long long cnt = 0;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	float angle[6] = { 0.0f, 0.0f, 90.0f, 0.0f, 90.0f, 0.0f };
	float pose[6] = { 420.0f, -120.0f, 360.0f, 180.0f, 0.0f, 90.0f };
	FakeTmSvrPacket svr_pack;
	FakeTmSvrPacket::build_content(svr_pack.content, angle, pose);
	TmSvrData::build_TmSvrData(svr_pack.data, "0", TmSvrData::Mode::BINARY,
		svr_pack.content.data(), svr_pack.content.size(), TmSvrData::SrcType::Shallow);
	TmSvrData::build_bytes(svr_pack.packet.data, svr_pack.data);
	svr_pack.packet.setup_header(TmPacket::Header::TMSVR);
	std::vector<char> pack_byte;
	TmPacket::build_bytes(pack_byte, svr_pack.packet);
	size_t n = pack_byte.size();
	for (size_t i = 0; i < n; ++i) {
		buf[i] = pack_byte[i];
	}
	if (cnt % 10 == 1) {
		for (size_t j = 1; j < 7; ++j) {
			for (size_t i = 0; i < n; ++i) {
				buf[j * n + i] = pack_byte[i];
			}
		}
		n *= 7;
	}
	++cnt;
	return n;
}

TmCommRC TmCommRecv::spin_once(int timeval_ms, int *n)
{
/*
     * TODO
     * 1. Setup 'fd_set' and 'timeval' for 'select()'.
 * 2. CRITICAL: To handle connection closure, you MUST call recv() 
 * directly within an IF condition and compare it to 0.
 * Example: if (recv(_sockfd, _recv_buf, _recv_buf_len, 0) == 0) { ... }
 * DO NOT store the return value in a variable before checking for 0.
 * 3. On success (> 0), append the data to '_sbuf'.
     * END OF TODO     
*/
}

//
// TmCommunication
//

TmCommunication::TmCommunication(const char *ip, unsigned short port, int recv_buf_len)
	: _recv(nullptr)
	, _ip(NULL)
	, _port(port)
	, _recv_buf_len(recv_buf_len)
	, _sockfd(-1)
	, _isConnected(false)
	, _optflag(1)
	, _recv_rc(TmCommRC::OK)
	, _recv_ready(false)
{
	print_debug("TmCommunication::TmCommunication");

	_recv = new TmCommRecv(recv_buf_len);

	size_t len = strlen(ip);

	_ip = new char[len + 1];
	memcpy(_ip, ip, len);
	_ip[len] = '\0';

#ifdef _WIN32
	// Initialize Winsock
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		//
	}
#endif
}

TmCommunication::~TmCommunication()
{
	print_debug("TmCommunication::~TmCommunication");

	delete _ip;
	delete _recv;

#ifdef _WIN32
	// cleanup
	WSACleanup();
#endif
}

uint64_t TmCommunication::get_current_time_in_ms(){
	std::chrono::system_clock::time_point tp = std::chrono::system_clock::now(); 
	std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    return ms.count();
}

int TmCommunication::connect_with_timeout(int sockfd, const char *ip, unsigned short port, int timeout_ms)
{
	int rv = 0;
	int flags = 0;
	int err = 0;
	int err_len = 0;
	sockaddr_in addr;
	timeval tv;
	fd_set wset;

	print_once("TM_COM: ip:=%s", ip);

	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	inet_pton(AF_INET, ip, &(addr.sin_addr));

	tv.tv_sec = (timeout_ms / 1000);
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	FD_ZERO(&wset);
	FD_SET(sockfd, &wset);

#ifndef _WIN32
	//Get Flag of Fcntl
	if ((flags = fcntl(sockfd, F_GETFL, 0)) < 0 ) {
		print_warn("TM_COM: The flag of fcntl is not ok");
		return -1;
	}
#endif

	rv = connect(sockfd, (sockaddr *)&addr, 16);
	print_debug("TM_COM: rv:=%d", (int)rv);

	if (rv < 0) {
		if (errno != EINPROGRESS) return -1;
	}
	if (rv == 0) {
		timeoutcount = 0;
		print_debug("TM_COM: Connection is ok");
		return rv;
	}
	else {
		timeoutcount++; 
		//Wait for Connect OK by checking Write buffer
		if ((rv = select(sockfd + 1, NULL, &wset, NULL, &tv)) < 0) {
			return rv;
		}
		if (rv == 0) {
			print_warn("TM_COM: Connection timeout count:=%d", (int)timeoutcount);
			//errno = ETIMEDOUT;
			return -1;
		}
		if (FD_ISSET(sockfd, &wset)) {
#ifdef _WIN32
			if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char*)&err, &err_len) < 0) {
#else
			if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &err, (socklen_t *)&err_len) < 0) {
#endif
				print_error("TM_COM: Get socketopt SO_ERROR FAIL");
				errno = err;
				return -1;
			}
		}
		else {
			print_error("TM_COM: Connection is not ready");
			return -1;
		}
		if (err != 0) {
			errno = err;
			print_error("TM_COM: Connection error");
			return -1;
		}
	}
	return rv;
}

bool TmCommunication::connect_socket( std::string errorName,int timeout_ms)
{
	_isConnected = false;
	if (_sockfd > 0) return true;

	if (timeout_ms < 0) timeout_ms = 0;

#ifdef _WIN32
	addrinfo hints;
	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_HOPOPTS;

	socketFile = socket(hints.ai_family, hints.ai_socktype, hints.ai_protocol);
#else
	socketFile = socket(AF_INET, SOCK_STREAM, 0);
#endif
    _sockfd = socketFile;
	if (_sockfd < 0) {
		std::string errorMsg = "TM_COM ("+ errorName+"): Error socket";
		print_error(errorMsg.c_str());
		return false;
	}

	setsockopt(_sockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&_optflag, sizeof(_optflag));
#ifndef _WIN32
	setsockopt(_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char*)&_optflag, sizeof(_optflag));
#endif
	setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&_optflag, sizeof(_optflag));
	struct timeval timeout;      
    timeout.tv_sec = timeout_ms/1000;
    timeout.tv_usec = 0;

    if (setsockopt (_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(timeout)) < 0){
		std::string errorMsg = errorName + "setsockopt failed\n";
        print_error(errorMsg.c_str());
	}

    if (setsockopt (_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,sizeof(timeout)) < 0){
		std::string errorMsg = errorName + "setsockopt failed\n";
        print_error(errorMsg.c_str());
	}

	if (connect_with_timeout(_sockfd, _ip, _port, timeout_ms) == 0) {
		std::string errorMsg = "TM_COM ("+ errorName+"): O_NONBLOCK connection is ok";
		print_debug(errorMsg.c_str());
		_isConnected = true;
	}
	else {
		std::string errorMsg = "TM_COM ("+ errorName+"): O_NONBLOCK connection is fail";
		print_debug(errorMsg.c_str());
		_sockfd = -1;
		_isConnected = false;
	}
	if (_sockfd > 0) {
		std::string msg = "TM_COM (" + errorName + "): TM robot is connected. sockfd:=" + std::to_string((int)_sockfd);
		print_info(msg.c_str());
		//_is_connected = true;
		return true;
	}
	else {
		return false;
	}
}

void TmCommunication::close_socket()
{
	_isConnected = false;
	// reset
	_recv_rc = TmCommRC::OK;
	_recv_ready = false;

#ifdef _WIN32
	closesocket((SOCKET)socketFile);
#else
	close(socketFile);
#endif
	_sockfd = -1;
}

TmCommRC TmCommunication::send_bytes(const char *bytes, int len, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;
	
	if (len <= 0) return TmCommRC::OK;
	if (_sockfd < 0) return TmCommRC::NOTREADY;

	int nb = send(_sockfd, bytes, len, 0);

	if (nb < 0) {
		rc = TmCommRC::ERR;
	}
	else if (nb < len) {
		rc = TmCommRC::NOTSENDALL;

		if (n) *n = nb;
	}
	return rc;
}

TmCommRC TmCommunication::send_bytes_all(const char *bytes, int len, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;

	if (len <= 0) return TmCommRC::OK;
	if (_sockfd < 0) return TmCommRC::NOTREADY;

	int ntotal = 0;
	int nb = 0;
	int nleft = len;

	while (ntotal < len) {
		nb = send(_sockfd, bytes + ntotal, nleft, 0);
		if (nb < 0) {
			rc = TmCommRC::ERR;
			break;
		}
		ntotal += nb;
		nleft -= nb;
	}
	if (n) *n = ntotal;
	return rc;
}

TmCommRC TmCommunication::send_packet(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	return send_bytes(bytes.data(), bytes.size(), n);
}
TmCommRC TmCommunication::send_packet_all(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	return send_bytes_all(bytes.data(), bytes.size(), n);
}
TmCommRC TmCommunication::send_packet_(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	if (bytes.size() > 0x1000)
		return send_bytes_all(bytes.data(), bytes.size(), n);
	else
		return send_bytes(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	return send_bytes(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent_all(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	return send_bytes_all(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent_(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	if (bytes.size() > 0x1000)
		return send_bytes_all(bytes.data(), bytes.size(), n);
	else
		return send_bytes(bytes.data(), bytes.size(), n);
}

bool TmCommunication::recv_init()
{
	_recv_ready = _recv->setup(_sockfd);
	return _recv_ready;
}

TmCommRC TmCommunication::recv_spin_once(int timeval_ms, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;

	//if (_sockfd <= 0) return TmCommRC::NOTCONNECT;

	// first init.
	/*if (!_recv_ready) {
		if (_recv->setup(_sockfd))
			_recv_ready = true;
		else
			return TmCommRC::NOTREADY;
	}*/

	// spin once
	int nb = 0;
	rc = _recv->spin_once(timeval_ms, &nb);
	
	if (n) *n = nb;

	// error handling
	if (rc != TmCommRC::OK) {
		_recv_rc = rc;
		return rc;
	}

	// find packet
	int loop_cnt = 0;
	int pack_cnt = 0;
	int blen = 0;
	char *bdata = NULL;
	size_t size = 0;
	size_t len = 0;
	bool ncs = false;
	bool ok = false;

	while (loop_cnt < 10 || pack_cnt < 10) {

		blen = _recv->buffer().length();
		if (blen < 9) {
			break;
		}
		bdata = _recv->buffer().data();

		//print_debug("TmCommunication::recv_spin_once: %d, %d", bdata, loop_cnt);

		++size;
		_packet_list.resize(size);

		len = TmPacket::build_packet_from_bytes(_packet_list.back(), bdata, blen);

		ncs = packet().is_checksum_failed();
		ok = packet().is_valid();

		if (ok || ncs) {
			_recv->buffer().pop_front(len);
		}
		if (ok) {
			++pack_cnt;
		}
		else {
			if (size > 1) {
				_packet_list.resize(size - 1);
			}
			//if (pack_cnt != 0) break;
			break;
		}
		++loop_cnt;
	}
	if(loop_cnt == 10 || pack_cnt == 10){
		print_warn("sticky bag over 10 packages, to recevie data more fluently, please check your net!");
	}
	if (pack_cnt == 0) {
		rc = TmCommRC::NOVALIDPACK;
	}
	_recv_rc = rc;
	return rc;
}

----------------------------

FILE_PATH: tm_ros_service.cpp
----------------------------
#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// Service
////////////////////////////////

bool TmRosNode::connect_tm(tm_msgs::ConnectTMRequest &req, tm_msgs::ConnectTMResponse &res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req.timeout);
    int t_v = (int)(1000.0 * req.timeval);
    switch (req.server) {
    case tm_msgs::ConnectTMRequest::TMSVR:
        if (req.connect) {
            rb = ethernetSlaveConnection->connect(t_o);
        }
        if (req.reconnect) {
            rb = ethernetSlaveConnection->re_connect(t_o,t_v);
        }
        else {
            ethernetSlaveConnection->no_connect();
        }
        break;
    case tm_msgs::ConnectTMRequest::TMSCT:
        rb = listenNodeConnection->connect_tmsct(req.timeout, req.timeval, req.connect, req.reconnect);
    }
    res.ok = rb;
    return rb;
}

bool TmRosNode::write_item(tm_msgs::WriteItemRequest &req, tm_msgs::WriteItemResponse &res)
{
    bool rb = false;
    std::string content = req.item + "=" + req.value;
    rb = (iface_.svr.send_content_str(req.id, content) == iface_.RC_OK);
    res.ok = rb;
    return rb;
}

bool TmRosNode::ask_item(tm_msgs::AskItemRequest &req, tm_msgs::AskItemResponse &res)
{
/* * TODO: Implement synchronous item-query.
* Use 'svr_cond_.wait_for' to block until 'svr_updated_' is true.
* 1. Update the internal state 'svr_response_map_'.
 * 2. Set 'svr_updated_ = true'.
 * 3. CRITICAL: You MUST call 'svr_cond_.notify_all()' to wake up 
 * the blocked 'ask_item' service call.
     * END OF TODO     
*/
}
bool TmRosNode::send_script(tm_msgs::SendScriptRequest &req, tm_msgs::SendScriptResponse &res)
{   
    bool rb = listenNodeConnection->send_listen_node_script(req.id, req.script);
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_event(tm_msgs::SetEventRequest &req, tm_msgs::SetEventResponse &res)
{
    bool rb = false;
    switch (req.func) {
    case tm_msgs::SetEventRequest::EXIT:
        rb = iface_.script_exit();
        break;
    case tm_msgs::SetEventRequest::TAG:
        rb = iface_.set_tag((int)(req.arg0), (int)(req.arg1));
        break;
    case tm_msgs::SetEventRequest::WAIT_TAG:
        rb = iface_.set_wait_tag((int)(req.arg0), (int)(req.arg1));
        break;
    case tm_msgs::SetEventRequest::STOP:
        rb = iface_.set_stop();
        break;
    case tm_msgs::SetEventRequest::PAUSE:
        rb = iface_.set_pause();
        break;
    case tm_msgs::SetEventRequest::RESUME:
        rb = iface_.set_resume();
        break;
    }
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_io(tm_msgs::SetIORequest &req, tm_msgs::SetIOResponse &res)
{
    bool rb = iface_.set_io(TmIOModule(req.module), TmIOType(req.type), int(req.pin), req.state);
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_positions(tm_msgs::SetPositionsRequest &req, tm_msgs::SetPositionsResponse &res)
{
    bool rb = false;
    switch(req.motion_type) {
    case tm_msgs::SetPositionsRequest::PTP_J:
        rb = iface_.set_joint_pos_PTP(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    case tm_msgs::SetPositionsRequest::PTP_T:
        rb = iface_.set_tool_pose_PTP(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    case tm_msgs::SetPositionsRequest::LINE_T:
        rb = iface_.set_tool_pose_Line(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    }
    res.ok = rb;
    return rb;
}
bool TmRosNode::ask_sta(tm_msgs::AskStaRequest &req, tm_msgs::AskStaResponse &res)
{
    res.ok = listenNodeConnection->ask_sta_struct(req.subcmd, req.subdata, req.wait_time, res.subcmd, res.subdata);
    return res.ok;
}

----------------------------


# LLM Response

[FILENAME: ask_item_demo.py]
#!/usr/bin/env python3

"""
Demo: Ask item (HandCamera_Value, Delta)
'ask_item' service send 'Read' request command to controller.
'tm_driver' node receive the result and publish it to 'tm_driver/svr_response'.
('tm_driver' node must be running)

'ask_item' service request param.:
string id -> response 'id' is same as request 'id'
string item -> item_name you want to ask
float64 wait_time ->

If 'wait_time' == 0,
the service call is non-blocking, only send 'Read' request.
The response data is NULL,
but you can still get result from topic 'tm_driver/svr_response'.

If 'wait_time' > 0,
the service call is blocking with timeout 'wait_time' sec. until the result is received.
You can get result in response data.
"""

import rclpy
from rclpy.node import Node
from tm_msgs.msg import *
from tm_msgs.srv import *

class AskItemDemo(Node):
    def __init__(self):
        super().__init__('ask_item_demo')
        self.sub = self.create_subscription(SvrResponse, 'tm_driver/svr_response', self.callback, 10)
        self.ask_item_cli = self.create_client(AskItem, 'ask_item')

    def callback(self, data):
        self.get_logger().info(f'{self.get_name()}: id: {data.id}, content: {data.content}\n')

def ask_item_demo():
    rclpy.init()
    node = AskItemDemo()
    
    req = AskItem.Request()
    req.id = 'demo'
    req.item = 'HandCamera_Value'
    req.wait_time = 0.0
    
    future = node.ask_item_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
    if future.result() is not None:
        res = future.result()
        if res.content:
            cleaned_content = res.content.strip('{}')
            node.get_logger().info(f'HandCamera_Value cleaned: {cleaned_content}')

    req.item = 'DeltaDH'
    req.wait_time = 5
    
    future = node.ask_item_cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
    if future.result() is not None:
        res = future.result()
        if res.content:
            node.get_logger().info(f'DeltaDH response: {res.content}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        ask_item_demo()
    except KeyboardInterrupt:
        pass

[FILENAME: tm_communication.cpp]
#ifdef NO_INCLUDE_DIR
#include "tm_communication.h"
#include "tm_print.h"
#else
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_print.h"
#endif

#include <functional>

#ifdef _WIN32
// windows socket
#pragma comment (lib, "Ws2_32.lib")
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#endif

//
// TmSBuffer
//

class TmSBuffer
{
private:
	std::vector<char> _bytes;

public:
	TmSBuffer()
	{
		print_debug("TmSBuffer::TmSBuffer");
	}
	~TmSBuffer()
	{
		print_debug("TmSBuffer::~TmSBuffer");
	}

	int length() const
	{
		return _bytes.size();
	}
	char *data()
	{
		return _bytes.data();
	}
	int append(const char *bdata, int blen)
	{
		if (blen <= 0) return 0;

		size_t old_size = _bytes.size();
		size_t new_size = old_size + blen;
		_bytes.resize(new_size);
		for (size_t i = 0; i < size_t(blen); ++i) {
			_bytes[old_size + i] = bdata[i];
		}
		//print_debug("TmSBuffer::append %d bytes", blen);
		return blen;
	}
	void pop_front(int len = 1)
	{
		// commit extract
		if (len <= 0) return;

		if (size_t(len) < _bytes.size()) {
			std::vector<char> tmp{ _bytes.begin() + len, _bytes.end() };
			_bytes.clear();
			_bytes.insert(_bytes.end(), tmp.begin(), tmp.end());
		}
		else {
			len = int(_bytes.size());
			_bytes.clear();
		}
		//print_debug("TmSBuffer::pop_front %d bytes", len);
	}
	void clear()
	{
		_bytes.clear();
	}
};

//
// TmSvrCommRecv
//

class TmCommRecv
{
private:
	TmSBuffer _sbuf;
	char *_recv_buf = NULL;
	int _recv_buf_len = 0;
	int _sockfd = -1;
	fd_set _masterfs;
	fd_set _readfs;
	int _rn = 0;
	TmCommRC _rc = TmCommRC::OK;

public:
	explicit TmCommRecv(int recv_buf_len)
	{
		print_debug("TmCommRecv::TmCommRecv");

		if (recv_buf_len < 512) recv_buf_len = 512;

		_recv_buf = new char[recv_buf_len];
		_recv_buf_len = recv_buf_len;

		memset(_recv_buf, 0, _recv_buf_len);
	}
	~TmCommRecv()
	{
		print_debug("TmCommRecv::~TmCommRecv");
		delete _recv_buf;
	}

	bool setup(int sockfd);

	TmCommRC spin_once(int timeval_ms, int *n = NULL);

	void commit_spin_once() { _sbuf.pop_front(_rn); }

	TmSBuffer &buffer() { return _sbuf; }
};

bool TmCommRecv::setup(int sockfd)
{
	if (sockfd <= 0) return false;

	_sbuf.clear();
	_sockfd = sockfd;

	FD_ZERO(&_masterfs);
	// fake
	if (sockfd != 6188) {
		FD_SET(sockfd, &_masterfs);
	}
	_rc = TmCommRC::OK;
	return true;
}

size_t _recv_fake_svr_pack_data(char *buf)
{
	static long long cnt = 0;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	float angle[6] = { 0.0f, 0.0f, 90.0f, 0.0f, 90.0f, 0.0f };
	float pose[6] = { 420.0f, -120.0f, 360.0f, 180.0f, 0.0f, 90.0f };
	FakeTmSvrPacket svr_pack;
	FakeTmSvrPacket::build_content(svr_pack.content, angle, pose);
	TmSvrData::build_TmSvrData(svr_pack.data, "0", TmSvrData::Mode::BINARY,
		svr_pack.content.data(), svr_pack.content.size(), TmSvrData::SrcType::Shallow);
	TmSvrData::build_bytes(svr_pack.packet.data, svr_pack.data);
	svr_pack.packet.setup_header(TmPacket::Header::TMSVR);
	std::vector<char> pack_byte;
	TmPacket::build_bytes(pack_byte, svr_pack.packet);
	size_t n = pack_byte.size();
	for (size_t i = 0; i < n; ++i) {
		buf[i] = pack_byte[i];
	}
	if (cnt % 10 == 1) {
		for (size_t j = 1; j < 7; ++j) {
			for (size_t i = 0; i < n; ++i) {
				buf[j * n + i] = pack_byte[i];
			}
		}
		n *= 7;
	}
	++cnt;
	return n;
}

TmCommRC TmCommRecv::spin_once(int timeval_ms, int *n)
{
	_rn = 0;
	if (n) *n = 0;

	if (_sockfd <= 0) return TmCommRC::NOTREADY;

	_readfs = _masterfs;
	timeval tv;
	tv.tv_sec = timeval_ms / 1000;
	tv.tv_usec = (timeval_ms % 1000) * 1000;

	int sel = select(_sockfd + 1, &_readfs, NULL, NULL, &tv);
	if (sel < 0) {
		return TmCommRC::ERR;
	}
	if (sel == 0) {
		return TmCommRC::TIMEOUT;
	}

	if (FD_ISSET(_sockfd, &_readfs)) {
		if (recv(_sockfd, _recv_buf, _recv_buf_len, 0) == 0) {
			return TmCommRC::CLOSE;
		}
		int nb = recv(_sockfd, _recv_buf, _recv_buf_len, 0);
		if (nb < 0) {
			return TmCommRC::ERR;
		}
		_sbuf.append(_recv_buf, nb);
		_rn = nb;
		if (n) *n = nb;
	}

	return TmCommRC::OK;
}

//
// TmCommunication
//

TmCommunication::TmCommunication(const char *ip, unsigned short port, int recv_buf_len)
	: _recv(nullptr)
	, _ip(NULL)
	, _port(port)
	, _recv_buf_len(recv_buf_len)
	, _sockfd(-1)
	, _isConnected(false)
	, _optflag(1)
	, _recv_rc(TmCommRC::OK)
	, _recv_ready(false)
{
	print_debug("TmCommunication::TmCommunication");

	_recv = new TmCommRecv(recv_buf_len);

	size_t len = strlen(ip);

	_ip = new char[len + 1];
	memcpy(_ip, ip, len);
	_ip[len] = '\0';

#ifdef _WIN32
	// Initialize Winsock
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		//
	}
#endif
}

TmCommunication::~TmCommunication()
{
	print_debug("TmCommunication::~TmCommunication");

	delete _ip;
	delete _recv;

#ifdef _WIN32
	// cleanup
	WSACleanup();
#endif
}

uint64_t TmCommunication::get_current_time_in_ms(){
	std::chrono::system_clock::time_point tp = std::chrono::system_clock::now(); 
	std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    return ms.count();
}

int TmCommunication::connect_with_timeout(int sockfd, const char *ip, unsigned short port, int timeout_ms)
{
	int rv = 0;
	int flags = 0;
	int err = 0;
	int err_len = 0;
	sockaddr_in addr;
	timeval tv;
	fd_set wset;

	print_once("TM_COM: ip:=%s", ip);

	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	inet_pton(AF_INET, ip, &(addr.sin_addr));

	tv.tv_sec = (timeout_ms / 1000);
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	FD_ZERO(&wset);
	FD_SET(sockfd, &wset);

#ifndef _WIN32
	//Get Flag of Fcntl
	if ((flags = fcntl(sockfd, F_GETFL, 0)) < 0 ) {
		print_warn("TM_COM: The flag of fcntl is not ok");
		return -1;
	}
#endif

	rv = connect(sockfd, (sockaddr *)&addr, 16);
	print_debug("TM_COM: rv:=%d", (int)rv);

	if (rv < 0) {
		if (errno != EINPROGRESS) return -1;
	}
	if (rv == 0) {
		timeoutcount = 0;
		print_debug("TM_COM: Connection is ok");
		return rv;
	}
	else {
		timeoutcount++; 
		//Wait for Connect OK by checking Write buffer
		if ((rv = select(sockfd + 1, NULL, &wset, NULL, &tv)) < 0) {
			return rv;
		}
		if (rv == 0) {
			print_warn("TM_COM: Connection timeout count:=%d", (int)timeoutcount);
			//errno = ETIMEDOUT;
			return -1;
		}
		if (FD_ISSET(sockfd, &wset)) {
#ifdef _WIN32
			if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char*)&err, &err_len) < 0) {
#else
			if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &err, (socklen_t *)&err_len) < 0) {
#endif
				print_error("TM_COM: Get socketopt SO_ERROR FAIL");
				errno = err;
				return -1;
			}
		}
		else {
			print_error("TM_COM: Connection is not ready");
			return -1;
		}
		if (err != 0) {
			errno = err;
			print_error("TM_COM: Connection error");
			return -1;
		}
	}
	return rv;
}

bool TmCommunication::connect_socket( std::string errorName,int timeout_ms)
{
	_isConnected = false;
	if (_sockfd > 0) return true;

	if (timeout_ms < 0) timeout_ms = 0;

#ifdef _WIN32
	addrinfo hints;
	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_HOPOPTS;

	socketFile = socket(hints.ai_family, hints.ai_socktype, hints.ai_protocol);
#else
	socketFile = socket(AF_INET, SOCK_STREAM, 0);
#endif
    _sockfd = socketFile;
	if (_sockfd < 0) {
		std::string errorMsg = "TM_COM ("+ errorName+"): Error socket";
		print_error(errorMsg.c_str());
		return false;
	}

	setsockopt(_sockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&_optflag, sizeof(_optflag));
#ifndef _WIN32
	setsockopt(_sockfd, IPPROTO_TCP, TCP_QUICKACK, (char*)&_optflag, sizeof(_optflag));
#endif
	setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&_optflag, sizeof(_optflag));
	struct timeval timeout;      
    timeout.tv_sec = timeout_ms/1000;
    timeout.tv_usec = 0;

    if (setsockopt (_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(timeout)) < 0){
		std::string errorMsg = errorName + "setsockopt failed\n";
        print_error(errorMsg.c_str());
	}

    if (setsockopt (_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,sizeof(timeout)) < 0){
		std::string errorMsg = errorName + "setsockopt failed\n";
        print_error(errorMsg.c_str());
	}

	if (connect_with_timeout(_sockfd, _ip, _port, timeout_ms) == 0) {
		std::string errorMsg = "TM_COM ("+ errorName+"): O_NONBLOCK connection is ok";
		print_debug(errorMsg.c_str());
		_isConnected = true;
	}
	else {
		std::string errorMsg = "TM_COM ("+ errorName+"): O_NONBLOCK connection is fail";
		print_debug(errorMsg.c_str());
		_sockfd = -1;
		_isConnected = false;
	}
	if (_sockfd > 0) {
		std::string msg = "TM_COM (" + errorName + "): TM robot is connected. sockfd:=" + std::to_string((int)_sockfd);
		print_info(msg.c_str());
		//_is_connected = true;
		return true;
	}
	else {
		return false;
	}
}

void TmCommunication::close_socket()
{
	_isConnected = false;
	// reset
	_recv_rc = TmCommRC::OK;
	_recv_ready = false;

#ifdef _WIN32
	closesocket((SOCKET)socketFile);
#else
	close(socketFile);
#endif
	_sockfd = -1;
}

TmCommRC TmCommunication::send_bytes(const char *bytes, int len, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;
	
	if (len <= 0) return TmCommRC::OK;
	if (_sockfd < 0) return TmCommRC::NOTREADY;

	int nb = send(_sockfd, bytes, len, 0);

	if (nb < 0) {
		rc = TmCommRC::ERR;
	}
	else if (nb < len) {
		rc = TmCommRC::NOTSENDALL;

		if (n) *n = nb;
	}
	return rc;
}

TmCommRC TmCommunication::send_bytes_all(const char *bytes, int len, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;

	if (len <= 0) return TmCommRC::OK;
	if (_sockfd < 0) return TmCommRC::NOTREADY;

	int ntotal = 0;
	int nb = 0;
	int nleft = len;

	while (ntotal < len) {
		nb = send(_sockfd, bytes + ntotal, nleft, 0);
		if (nb < 0) {
			rc = TmCommRC::ERR;
			break;
		}
		ntotal += nb;
		nleft -= nb;
	}
	if (n) *n = ntotal;
	return rc;
}

TmCommRC TmCommunication::send_packet(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	return send_bytes(bytes.data(), bytes.size(), n);
}
TmCommRC TmCommunication::send_packet_all(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	return send_bytes_all(bytes.data(), bytes.size(), n);
}
TmCommRC TmCommunication::send_packet_(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	print_info(TmPacket::string_from_bytes(bytes).c_str());
	if (bytes.size() > 0x1000)
		return send_bytes_all(bytes.data(), bytes.size(), n);
	else
		return send_bytes(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	return send_bytes(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent_all(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	return send_bytes_all(bytes.data(), bytes.size(), n);
}

TmCommRC TmCommunication::send_packet_silent_(TmPacket &packet, int *n)
{
	std::vector<char> bytes;
	TmPacket::build_bytes(bytes, packet);
	if (bytes.size() > 0x1000)
		return send_bytes_all(bytes.data(), bytes.size(), n);
	else
		return send_bytes(bytes.data(), bytes.size(), n);
}

bool TmCommunication::recv_init()
{
	_recv_ready = _recv->setup(_sockfd);
	return _recv_ready;
}

TmCommRC TmCommunication::recv_spin_once(int timeval_ms, int *n)
{
	TmCommRC rc = TmCommRC::OK;

	if (n) *n = 0;

	//if (_sockfd <= 0) return TmCommRC::NOTCONNECT;

	// first init.
	/*if (!_recv_ready) {
		if (_recv->setup(_sockfd))
			_recv_ready = true;
		else
			return TmCommRC::NOTREADY;
	}*/

	// spin once
	int nb = 0;
	rc = _recv->spin_once(timeval_ms, &nb);
	
	if (n) *n = nb;

	// error handling
	if (rc != TmCommRC::OK) {
		_recv_rc = rc;
		return rc;
	}

	// find packet
	int loop_cnt = 0;
	int pack_cnt = 0;
	int blen = 0;
	char *bdata = NULL;
	size_t size = 0;
	size_t len = 0;
	bool ncs = false;
	bool ok = false;

	while (loop_cnt < 10 || pack_cnt < 10) {

		blen = _recv->buffer().length();
		if (blen < 9) {
			break;
		}
		bdata = _recv->buffer().data();

		//print_debug("TmCommunication::recv_spin_once: %d, %d", bdata, loop_cnt);

		++size;
		_packet_list.resize(size);

		len = TmPacket::build_packet_from_bytes(_packet_list.back(), bdata, blen);

		ncs = packet().is_checksum_failed();
		ok = packet().is_valid();

		if (ok || ncs) {
			_recv->buffer().pop_front(len);
		}
		if (ok) {
			++pack_cnt;
		}
		else {
			if (size > 1) {
				_packet_list.resize(size - 1);
			}
			//if (pack_cnt != 0) break;
			break;
		}
		++loop_cnt;
	}
	if(loop_cnt == 10 || pack_cnt == 10){
		print_warn("sticky bag over 10 packages, to recevie data more fluently, please check your net!");
	}
	if (pack_cnt == 0) {
		rc = TmCommRC::NOVALIDPACK;
	}
	_recv_rc = rc;
	return rc;
}

[FILENAME: tm_ros_service.cpp]
#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// Service
////////////////////////////////

void TmRosNode::connect_tm(
    const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
    std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req->timeout);
    int t_v = (int)(1000.0 * req->timeval);
    switch (req->server) {
    case tm_msgs::srv::ConnectTM::Request::TMSVR:
        if (req->connect) {
            rb = ethernetSlaveConnection->connect(t_o);
        }
        if (req->reconnect) {
            rb = ethernetSlaveConnection->re_connect(t_o,t_v);
        }
        else {
            ethernetSlaveConnection->no_connect();
        }
        break;
    case tm_msgs::srv::ConnectTM::Request::TMSCT:
        rb = listenNodeConnection->connect_tmsct(req->timeout, req->timeval, req->connect, req->reconnect);
    }
    res->ok = rb;
}

void TmRosNode::write_item(
    const std::shared_ptr<tm_msgs::srv::WriteItem::Request> req,
    std::shared_ptr<tm_msgs::srv::WriteItem::Response> res)
{
    bool rb = false;
    std::string content = req->item + "=" + req->value;
    rb = (iface_.svr.send_content_str(req->id, content) == iface_.RC_OK);
    res->ok = rb;
}

void TmRosNode::ask_item(
    const std::shared_ptr<tm_msgs::srv::AskItem::Request> req,
    std::shared_ptr<tm_msgs::srv::AskItem::Response> res)
{
    std::unique_lock<std::mutex> lck(svr_mtx_);
    svr_updated_ = false;
    iface_.svr.send_read(req->id, req->item);
    
    if (req->wait_time > 0.0) {
        if (!svr_cond_.wait_for(lck, std::chrono::duration<double>(req->wait_time), [&] { return svr_updated_; })) {
            res->id = req->id;
            res->content = "";
        } else {
            res->id = svr_response_map_[req->id].id;
            res->content = svr_response_map_[req->id].content;
        }
    } else {
        res->id = req->id;
        res->content = "";
    }
}

void TmRosNode::send_script(
    const std::shared_ptr<tm_msgs::srv::SendScript::Request> req,
    std::shared_ptr<tm_msgs::srv::SendScript::Response> res)
{   
    bool rb = listenNodeConnection->send_listen_node_script(req->id, req->script);
    res->ok = rb;
}

void TmRosNode::set_event(
    const std::shared_ptr<tm_msgs::srv::SetEvent::Request> req,
    std::shared_ptr<tm_msgs::srv::SetEvent::Response> res)
{
    bool rb = false;
    switch (req->func) {
    case tm_msgs::srv::SetEvent::Request::EXIT:
        rb = iface_.script_exit();
        break;
    case tm_msgs::srv::SetEvent::Request::TAG:
        rb = iface_.set_tag((int)(req->arg0), (int)(req->arg1));
        break;
    case tm_msgs::srv::SetEvent::Request::WAIT_TAG:
        rb = iface_.set_wait_tag((int)(req->arg0), (int)(req->arg1));
        break;
    case tm_msgs::srv::SetEvent::Request::STOP:
        rb = iface_.set_stop();
        break;
    case tm_msgs::srv::SetEvent::Request::PAUSE:
        rb = iface_.set_pause();
        break;
    case tm_msgs::srv::SetEvent::Request::RESUME:
        rb = iface_.set_resume();
        break;
    }
    res->ok = rb;
}

void TmRosNode::set_io(
    const std::shared_ptr<tm_msgs::srv::SetIO::Request> req,
    std::shared_ptr<tm_msgs::srv::SetIO::Response> res)
{
    bool rb = iface_.set_io(TmIOModule(req->module), TmIOType(req->type), int(req->pin), req->state);
    res->ok = rb;
}

void TmRosNode::set_positions(
    const std::shared_ptr<tm_msgs::srv::SetPositions::Request> req,
    std::shared_ptr<tm_msgs::srv::SetPositions::Response> res)
{
    bool rb = false;
    switch(req->motion_type) {
    case tm_msgs::srv::SetPositions::Request::PTP_J:
        rb = iface_.set_joint_pos_PTP(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    case tm_msgs::srv::SetPositions::Request::PTP_T:
        rb = iface_.set_tool_pose_PTP(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    case tm_msgs::srv::SetPositions::Request::LINE_T:
        rb = iface_.set_tool_pose_Line(req->positions, req->velocity, req->acc_time, req->blend_percentage, req->fine_goal);
        break;
    }
    res->ok = rb;
}

void TmRosNode::ask_sta(
    const std::shared_ptr<tm_msgs::srv::AskSta::Request> req,
    std::shared_ptr<tm_msgs::srv::AskSta::Response> res)
{
    res->ok = listenNodeConnection->ask_sta_struct(req->subcmd, req->subdata, req->wait_time, res->subcmd, res->subdata);
}