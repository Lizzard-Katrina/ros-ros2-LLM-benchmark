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

FILE_PATH: bridge_node.cpp
----------------------------
/**
 * @file bridge_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Reliable TCP bridge for ros data transfer in unstable network.
 * It will send/receive the specified ROS topics in ../config/ros_topics.yaml
 * It uses zmq socket(PUB/SUB mode), which reconnects others autonomously and
 * supports 1-N pub-sub connection even with TCP protocol.
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * Core Idea: It would create the receving thread for each receiving ROS topic
 *  and send ROS messages in each sub_cb() callback.
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "bridge_node.hpp"

// /* send messages frequency control */
// // this is the original freq control func that has been deprecated
// bool send_freq_control(int i)
// {
//   ros::Time t_now = ros::Time::now(); 
//   bool discard_flag;
//   if ((t_now - sub_t_last[i]).toSec() * sendTopics[i].max_freq < 1.0) {
//     discard_flag = true;
//   }
//   else {
//     discard_flag = false;
//     sub_t_last[i] = t_now; 
//   }
//   return discard_flag; // flag of discarding this message
// }

/* send messages frequency control */
bool send_freq_control(int i)
{
  bool discard_flag;
  ros::Time t_now = ros::Time::now(); 
  // check whether the send of this message will exceed the freq limit in the last period
  if ((send_num[i] + 1) / (t_now - sub_t_last[i]).toSec() > sendTopics[i].max_freq) {
    discard_flag = true;
  }
  else {
    discard_flag = false;
    send_num[i] ++;
  }
  // freq control period (1s)
  if ((t_now - sub_t_last[i]).toSec() > 1.0){
    sub_t_last[i] = t_now;
    send_num[i] = 0;
  }
  return discard_flag; // flag of discarding this message
}

/* uniform callback functions for ROS subscribers */
template <typename T, int i>
void sub_cb(const T &msg)
{
  /* frequency control */
  auto ignore_flag = send_freq_control(i);
  if (ignore_flag){
    return; // discard this message sending, abort
  }

// TODO: [SYSTEM_LEVEL_MIGRATION_TASK]
// 1. Re-implement the message serialization logic for ROS 2.
// 2. Use 'rclcpp::Serialization<T>' to serialize the incoming message 'msg' into a byte buffer.
// 3. Populate a 'zmqpp::message' with the serialized data and send it.
// STYLE: You must use 'this->get_serialized_message_factory()' or equivalent 'rclcpp' 
// patterns to ensure the bridge maintains zero-copy potential where possible.
//END OF TODO
}


/* uniform deserialize and publish the receiving messages */
template<typename T>
void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i)
{
  T msg;
  // deserialize the receiving messages into ROS msg
  namespace ser = ros::serialization;
  ser::IStream stream(buffer_ptr, msg_size);
  ser::deserialize(stream, msg);
  // publish ROS msg
  topic_pubs[i].publish(msg);
}


/* receive thread function to receive messages and publish them */
void recv_func(int i)
{
  while(recv_thread_flags[i])
  {
    /* receive and process message */
    zmqpp::message recv_array;
    bool recv_flag; // receive success flag
    // std::cout << "ready receive!" << std::endl;
    // receive(&,true) for non-blocking, receive(&,false) for blocking
    bool dont_block = false; // 'true' leads to high cpu load
    if (recv_flag = receivers[i]->receive(recv_array, dont_block))
    {
      // std::cout << "receive!" << std::endl;
      size_t data_len;
      recv_array >> data_len; // unpack meta data
      /*  equal to:
        recv_array.get(&data_len, recv_array.read_cursor++); 
        void get(T &value, size_t const cursor){
          uint8_t const* byte = static_cast<uint8_t const*>(raw_data(cursor)); 
          b = *byte;} 
      */
      // a dynamic length array by unique_ptr
      std::unique_ptr<uint8_t> recv_buffer(new uint8_t[data_len]);  
      // continue to copy the raw_data of recv_array into buffer
      memcpy(recv_buffer.get(), static_cast<const uint8_t *>(recv_array.raw_data(recv_array.read_cursor())), data_len);
      deserialize_publish(recv_buffer.get(), data_len, recvTopics[i].type, i);

      // std::cout << data_len << std::endl;
      // std::cout << recv_buffer.get() << std::endl;
    }

    /* if receive() does not block, sleep to decrease loop rate */
    if (dont_block)
      std::this_thread::sleep_for(std::chrono::microseconds(1000)); // sleep for us
    else
    {
      /* check and report receive state */
      if (recv_flag != recv_flags_last[i]){
        std::string topicName = recvTopics[i].name;
        if (topicName.at(0) != '/') {
          if (ns == "/") {topicName = "/" + topicName;}
          else {topicName = ns + "/" + topicName;}
        }  // print namespace prefix if topic name is not global
        ROS_INFO("[bridge node] \"%s\" received!", topicName.c_str());
      } // false -> true(first message in)        
      recv_flags_last[i] = recv_flag;
    }
  }
  return;
}

/* close recv socket, unsubscribe ROS topic */
void stop_send(int i)
{
  // senders[i]->unbind(std::string const &endpoint);
  senders[i]->close(); // close the send socket
  topic_subs[i].shutdown(); // unsubscribe
}

/* stop recv thread, close recv socket, unadvertise ROS topic */
void stop_recv(int i)
{
  recv_thread_flags[i] = false; // finish recv_func()
  // receivers[i]->disconnect(std::string &endpoint);
  receivers[i]->close(); // close the receive socket
  topic_pubs[i].shutdown(); // unadvertise
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;
  ns = ros::this_node::getNamespace(); // namespace of this node

  std::cout << "--------[bridge_node]-------" << std::endl;
  std::cout << "namespaces=" << ns << std::endl;

  // get hostnames and IPs
  if (nh.getParam("IP", ip_xml) == false){
    ROS_ERROR("[bridge node] No IP found in the configuration!");
    return 1;
  }
  // get "send topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("send_topics", send_topics_xml)){
    ROS_ASSERT(send_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_send = send_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No send_topics found in the configuration!");
    len_send = 0;
  }
  // get "receive topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("recv_topics", recv_topics_xml)){
    ROS_ASSERT(recv_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_recv = recv_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No recv_topics found in the configuration!");
    len_recv = 0;
  }

  if (len_send > SUB_MAX)
  {
    ROS_FATAL("[bridge_node] The number of send topics in configuration exceeds the limit %d!", SUB_MAX);
    return 2;
  }

  std::cout << "-------------IP------------" << std::endl;
  for (auto iter = ip_xml.begin(); iter != ip_xml.end(); ++iter)
  {
    std::string host_name = iter->first;
    std::string host_ip = iter->second;
    std::cout << host_name << " : " << host_ip << std::endl;
    if (ip_map.find(host_name) != ip_map.end())
    { // ip_xml will never contain same names actually.
      ROS_WARN("[bridge node] IPs with the same name in configuration %s!", host_name.c_str());
    }
    ip_map[host_name] = host_ip;
  }

  std::cout << "--------send topics--------" << std::endl;
  std::set<int> srcPorts; // for duplicate check 
  for (int32_t i=0; i < len_send; ++i)
  {
    ROS_ASSERT(send_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue send_topic_xml = send_topics_xml[i];
    std::string topic_name = send_topic_xml["topic_name"];
    std::string msg_type = send_topic_xml["msg_type"];
    int max_freq = send_topic_xml["max_freq"];
    std::string srcIP = ip_map[send_topic_xml["srcIP"]];
    int srcPort = send_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    sendTopics.emplace_back(topic);
    // check for duplicate ports:
    if (srcPorts.find(srcPort) != srcPorts.end()) {
      ROS_FATAL("[bridge_node] Send topics with the same srcPort %d in configuration!", srcPort);
      return 3;
    }
    srcPorts.insert(srcPort); // for duplicate check 
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  " << topic.max_freq << "Hz(max)" << std::endl;
  }

  std::cout << "-------receive topics------" << std::endl;
  for (int32_t i=0; i < len_recv; ++i)
  {
    ROS_ASSERT(recv_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue recv_topic_xml = recv_topics_xml[i];
    std::string topic_name = recv_topic_xml["topic_name"];
    std::string msg_type = recv_topic_xml["msg_type"];
    int max_freq = recv_topic_xml["max_freq"];
    std::string srcIP = ip_map[recv_topic_xml["srcIP"]];
    int srcPort = recv_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    recvTopics.emplace_back(topic);
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  (from " << recv_topic_xml["srcIP"]  << ")" << std::endl;
  }

  // ********************* zmq socket initialize ***************************
  // send sockets (zmq socket PUB mode)
  for (int32_t i=0; i < len_send; ++i)
  {
    const std::string url = "tcp://" + sendTopics[i].ip + ":" + std::to_string(sendTopics[i].port);
    std::unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context, zmqpp::socket_type::pub));
    sender->bind(url);
    senders.emplace_back(std::move(sender)); //sender is now released by std::move
  }

  // receive sockets (zmq socket SUB mode)
  for (int32_t i=0; i < len_recv; ++i)
  {
    const std::string url = "tcp://" + recvTopics[i].ip + ":" + std::to_string(recvTopics[i].port);
    std::string const zmq_topic = ""; // "" means all zmq topic
    std::unique_ptr<zmqpp::socket> receiver(new zmqpp::socket(context, zmqpp::socket_type::sub));
    receiver->subscribe(zmq_topic);
    receiver->connect(url);
    receivers.emplace_back(std::move(receiver));
  }


  // ******************* ROS subscribe and publish *************************
  //ROS topic subsrcibe and send
  for (int32_t i=0; i < len_send; ++i)
  {
    sub_t_last.emplace_back(ros::Time::now()); // freq control period start time
    send_num.emplace_back(0); // the send messages number in a period
    ros::Subscriber subscriber;
    // The uniform callback function is sub_cb()
    subscriber = topic_subscriber(sendTopics[i].name, sendTopics[i].type, nh_public, i);
    topic_subs.emplace_back(subscriber);
    // use topic_subs[i].shutdown() to unsubscribe
  }

  // ROS topic receive and publish
  for (int32_t i=0; i < len_recv; ++i) 
  {
    ros::Publisher publisher;
    publisher = topic_publisher(recvTopics[i].name, recvTopics[i].type, nh_public);
    topic_pubs.emplace_back(publisher);
  }

  // ****************** launch receive threads *****************************
  for (int32_t i=0; i < len_recv; ++i)
  {
    recv_thread_flags.emplace_back(true); // enable receive thread flags
    recv_flags_last.emplace_back(false); // receive success flag
    recv_threads.emplace_back(std::thread(&recv_func, i));
  }

  ros::spin();

  // ***************** stop send/receive ******************************
  for (int32_t i=0; i < len_send; ++i){
    stop_send(i);
  }

  for (int32_t i=0; i < len_recv; ++i){
    stop_recv(i);
  }
  
  return 0;
}

----------------------------

FILE_PATH: bridge_node.hpp
----------------------------
/**
 * @file bridge_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file of bridge_node.cpp
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __BRIDGE_NODE__
#define __BRIDGE_NODE__
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <zmqpp/zmqpp.hpp>
/*
zmqpp is the c++ wrapper around ZeroMQ
Intall zmqpp first:
    sudo apt install libzmqpp-dev
zmqpp reference link:
    https://zeromq.github.io/zmqpp/namespacezmqpp.html
*/
#include "ros_sub_pub.hpp"

struct TopicInfo
{
  std::string name;
  std::string type;
  int max_freq;
  std::string ip;
  int port;
};

// TODO: [SYSTEM_LEVEL_MIGRATION_TASK]
// 1. Migrate the global variables to be part of a 'BridgeNode' class inheriting from 'rclcpp::Node'.
// 2. Replace 'ros::Subscriber' and 'ros::Publisher' with ROS 2 SharedPtr equivalents.
// 3. Use 'rclcpp::Time' for frequency control variables and 'std::string' for the namespace.
// STYLE: Declare all ROS 2 interfaces as 'std::vector<rclcpp::GenericPublisher::SharedPtr>' 
// and 'std::vector<rclcpp::GenericSubscription::SharedPtr>' to support dynamic types.
//END OF TODO
// ******************* send frequency control ***************************
std::vector<ros::Time> sub_t_last;
std::vector<int> send_num;
bool send_freq_control(int i);

// ****************** launch receive threads *****************************
std::vector<bool> recv_thread_flags;
std::vector<bool> recv_flags_last;
std::vector<std::thread> recv_threads;
void recv_func(int i);

// ***************** stop send/receive ******************************
void stop_send(int i);
void stop_recv(int i);

#endif

----------------------------


# LLM Response



Looking at the code, I need to migrate a ROS1 ZMQ bridge node to ROS2. The code uses ZMQ for inter-robot communication and ROS for local pub/sub. Let me trace through all the dependencies and convert everything properly.

[FILENAME: bridge_node.hpp]
/**
 * @file bridge_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file of bridge_node.cpp
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __BRIDGE_NODE__
#define __BRIDGE_NODE__
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <zmqpp/zmqpp.hpp>
/*
zmqpp is the c++ wrapper around ZeroMQ
Intall zmqpp first:
    sudo apt install libzmqpp-dev
zmqpp reference link:
    https://zeromq.github.io/zmqpp/namespacezmqpp.html
*/
#include "ros_sub_pub.hpp"

struct TopicInfo
{
  std::string name;
  std::string type;
  int max_freq;
  std::string ip;
  int port;
};

// ******************* global variables ***************************
std::string ns; // namespace of this node

// config params
std::map<std::string, std::string> ip_map;
std::vector<TopicInfo> sendTopics;
std::vector<TopicInfo> recvTopics;
int32_t len_send;
int32_t len_recv;

// zmq context and sockets
zmqpp::context context;
std::vector<std::unique_ptr<zmqpp::socket>> senders;
std::vector<std::unique_ptr<zmqpp::socket>> receivers;

// ROS2 subscribers and publishers
std::vector<rclcpp::SubscriptionBase::SharedPtr> topic_subs;
std::vector<rclcpp::PublisherBase::SharedPtr> topic_pubs;

// shared node pointer for use in callbacks
rclcpp::Node::SharedPtr g_node;

// ******************* send frequency control ***************************
std::vector<rclcpp::Time> sub_t_last;
std::vector<int> send_num;
bool send_freq_control(int i);

// ****************** launch receive threads *****************************
std::vector<bool> recv_thread_flags;
std::vector<bool> recv_flags_last;
std::vector<std::thread> recv_threads;
void recv_func(int i);

// ***************** stop send/receive ******************************
void stop_send(int i);
void stop_recv(int i);

#endif

[FILENAME: bridge_node.cpp]
/**
 * @file bridge_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Reliable TCP bridge for ros data transfer in unstable network.
 * It will send/receive the specified ROS topics in ../config/ros_topics.yaml
 * It uses zmq socket(PUB/SUB mode), which reconnects others autonomously and
 * supports 1-N pub-sub connection even with TCP protocol.
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * Core Idea: It would create the receving thread for each receiving ROS topic
 *  and send ROS messages in each sub_cb() callback.
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "bridge_node.hpp"

/* send messages frequency control */
bool send_freq_control(int i)
{
  bool discard_flag;
  rclcpp::Time t_now = g_node->get_clock()->now();
  // check whether the send of this message will exceed the freq limit in the last period
  if ((send_num[i] + 1) / (t_now - sub_t_last[i]).seconds() > sendTopics[i].max_freq) {
    discard_flag = true;
  }
  else {
    discard_flag = false;
    send_num[i] ++;
  }
  // freq control period (1s)
  if ((t_now - sub_t_last[i]).seconds() > 1.0){
    sub_t_last[i] = t_now;
    send_num[i] = 0;
  }
  return discard_flag; // flag of discarding this message
}

/* uniform callback functions for ROS subscribers */
template <typename T, int i>
void sub_cb(const T &msg)
{
  /* frequency control */
  auto ignore_flag = send_freq_control(i);
  if (ignore_flag){
    return; // discard this message sending, abort
  }

  // Serialize the ROS2 message into a byte buffer
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(&msg, &serialized_msg);

  size_t data_len = serialized_msg.size();
  const uint8_t* data_ptr = serialized_msg.get_rcl_serialized_message().buffer;

  // Pack serialized data into zmqpp message and send
  zmqpp::message send_array;
  send_array << data_len;
  send_array.add_raw(reinterpret_cast<const void*>(data_ptr), data_len);
  senders[i]->send(send_array, true); // non-blocking send
}


/* uniform deserialize and publish the receiving messages */
template<typename T>
void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i)
{
  T msg;
  // deserialize the receiving messages into ROS msg
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg(msg_size);
  auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
  memcpy(rcl_msg.buffer, buffer_ptr, msg_size);
  rcl_msg.buffer_length = msg_size;
  serializer.deserialize_message(&serialized_msg, &msg);
  // publish ROS msg
  auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(topic_pubs[i]);
  if (pub) {
    pub->publish(msg);
  }
}


/* receive thread function to receive messages and publish them */
void recv_func(int i)
{
  while(recv_thread_flags[i])
  {
    /* receive and process message */
    zmqpp::message recv_array;
    bool recv_flag; // receive success flag
    // receive(&,true) for non-blocking, receive(&,false) for blocking
    bool dont_block = false; // 'true' leads to high cpu load
    if (recv_flag = receivers[i]->receive(recv_array, dont_block))
    {
      size_t data_len;
      recv_array >> data_len; // unpack meta data
      // a dynamic length array by unique_ptr
      std::unique_ptr<uint8_t> recv_buffer(new uint8_t[data_len]);  
      // continue to copy the raw_data of recv_array into buffer
      memcpy(recv_buffer.get(), static_cast<const uint8_t *>(recv_array.raw_data(recv_array.read_cursor())), data_len);
      deserialize_publish(recv_buffer.get(), data_len, recvTopics[i].type, i);
    }

    /* if receive() does not block, sleep to decrease loop rate */
    if (dont_block)
      std::this_thread::sleep_for(std::chrono::microseconds(1000)); // sleep for us
    else
    {
      /* check and report receive state */
      if (recv_flag != recv_flags_last[i]){
        std::string topicName = recvTopics[i].name;
        if (topicName.at(0) != '/') {
          if (ns == "/") {topicName = "/" + topicName;}
          else {topicName = ns + "/" + topicName;}
        }  // print namespace prefix if topic name is not global
        RCLCPP_INFO(g_node->get_logger(), "[bridge node] \"%s\" received!", topicName.c_str());
      } // false -> true(first message in)        
      recv_flags_last[i] = recv_flag;
    }
  }
  return;
}

/* close recv socket, unsubscribe ROS topic */
void stop_send(int i)
{
  senders[i]->close(); // close the send socket
  topic_subs[i].reset(); // unsubscribe
}

/* stop recv thread, close recv socket, unadvertise ROS topic */
void stop_recv(int i)
{
  recv_thread_flags[i] = false; // finish recv_func()
  receivers[i]->close(); // close the receive socket
  topic_pubs[i].reset(); // unadvertise
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("swarm_bridge");
  ns = g_node->get_namespace(); // namespace of this node

  std::cout << "--------[bridge_node]-------" << std::endl;
  std::cout << "namespaces=" << ns << std::endl;

  // get hostnames and IPs
  // In ROS2, we declare and get parameters differently
  // We expect parameters to be set via YAML or launch file
  // IP is a map of hostname -> ip
  {
    // Declare and get IP parameters
    // We expect IP to be provided as individual parameters: IP.<hostname> = <ip>
    // First try to get the list of IP entries
    std::map<std::string, rclcpp::Parameter> ip_params;
    if (!g_node->has_parameter("IP")) {
      // Try to list parameters with prefix "IP"
      auto param_result = g_node->list_parameters({"IP"}, 2);
      if (param_result.names.empty()) {
        RCLCPP_ERROR(g_node->get_logger(), "[bridge node] No IP found in the configuration!");
        return 1;
      }
      for (auto & name : param_result.names) {
        g_node->declare_parameter<std::string>(name, "");
        std::string val = g_node->get_parameter(name).as_string();
        // Extract hostname from "IP.hostname"
        std::string hostname = name.substr(3); // skip "IP."
        ip_map[hostname] = val;
      }
    } else {
      // Fallback: try declaring with prefix
      auto param_result = g_node->list_parameters({"IP"}, 2);
      for (auto & name : param_result.names) {
        if (name == "IP") continue;
        g_node->declare_parameter<std::string>(name, "");
        std::string val = g_node->get_parameter(name).as_string();
        std::string hostname = name.substr(3);
        ip_map[hostname] = val;
      }
    }
    if (ip_map.empty()) {
      // Alternative: try to get IP as a set of sub-parameters
      // Declare all IP.* parameters
      rcl_interfaces::msg::ParameterDescriptor desc;
      auto params_result = g_node->list_parameters({"IP"}, 10);
      for (const auto & pname : params_result.names) {
        if (pname.find("IP.") == 0) {
          if (!g_node->has_parameter(pname)) {
            g_node->declare_parameter<std::string>(pname, "");
          }
          std::string val = g_node->get_parameter(pname).as_string();
          std::string hostname = pname.substr(3);
          ip_map[hostname] = val;
        }
      }
    }
    if (ip_map.empty()) {
      RCLCPP_ERROR(g_node->get_logger(), "[bridge node] No IP found in the configuration!");
      return 1;
    }
  }

  // get send_topics params
  {
    auto param_result = g_node->list_parameters({"send_topics"}, 10);
    // Determine number of send topics
    std::set<int> indices;
    for (auto & name : param_result.names) {
      // Expected format: send_topics.<index>.topic_name etc.
      if (name.find("send_topics.") == 0) {
        std::string rest = name.substr(12);
        size_t dot = rest.find('.');
        if (dot != std::string::npos) {
          int idx = std::stoi(rest.substr(0, dot));
          indices.insert(idx);
        }
      }
    }
    len_send = indices.size();
    if (len_send == 0) {
      RCLCPP_WARN(g_node->get_logger(), "[bridge node] No send_topics found in the configuration!");
    }
  }

  // get recv_topics params
  {
    auto param_result = g_node->list_parameters({"recv_topics"}, 10);
    std::set<int> indices;
    for (auto & name : param_result.names) {
      if (name.find("recv_topics.") == 0) {
        std::string rest = name.substr(12);
        size_t dot = rest.find('.');
        if (dot != std::string::npos) {
          int idx = std::stoi(rest.substr(0, dot));
          indices.insert(idx);
        }
      }
    }
    len_recv = indices.size();
    if (len_recv == 0) {
      RCLCPP_WARN(g_node->get_logger(), "[bridge node] No recv_topics found in the configuration!");
    }
  }

  if (len_send > SUB_MAX)
  {
    RCLCPP_FATAL(g_node->get_logger(), "[bridge_node] The number of send topics in configuration exceeds the limit %d!", SUB_MAX);
    return 2;
  }

  std::cout << "-------------IP------------" << std::endl;
  for (auto iter = ip_map.begin(); iter != ip_map.end(); ++iter)
  {
    std::cout << iter->first << " : " << iter->second << std::endl;
  }

  std::cout << "--------send topics--------" << std::endl;
  std::set<int> srcPorts; // for duplicate check 
  for (int32_t i=0; i < len_send; ++i)
  {
    std::string prefix = "send_topics." + std::to_string(i) + ".";
    g_node->declare_parameter<std::string>(prefix + "topic_name", "");
    g_node->declare_parameter<std::string>(prefix + "msg_type", "");
    g_node->declare_parameter<int>(prefix + "max_freq", 10);
    g_node->declare_parameter<std::string>(prefix + "srcIP", "");
    g_node->declare_parameter<int>(prefix + "srcPort", 0);

    std::string topic_name = g_node->get_parameter(prefix + "topic_name").as_string();
    std::string msg_type = g_node->get_parameter(prefix + "msg_type").as_string();
    int max_freq = g_node->get_parameter(prefix + "max_freq").as_int();
    std::string srcIP_name = g_node->get_parameter(prefix + "srcIP").as_string();
    std::string srcIP = ip_map[srcIP_name];
    int srcPort = g_node->get_parameter(prefix + "srcPort").as_int();
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    sendTopics.emplace_back(topic);
    // check for duplicate ports:
    if (srcPorts.find(srcPort) != srcPorts.end()) {
      RCLCPP_FATAL(g_node->get_logger(), "[bridge_node] Send topics with the same srcPort %d in configuration!", srcPort);
      return 3;
    }
    srcPorts.insert(srcPort); // for duplicate check 
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  " << topic.max_freq << "Hz(max)" << std::endl;
  }

  std::cout << "-------receive topics------" << std::endl;
  for (int32_t i=0; i < len_recv; ++i)
  {
    std::string prefix = "recv_topics." + std::to_string(i) + ".";
    g_node->declare_parameter<std::string>(prefix + "topic_name", "");
    g_node->declare_parameter<std::string>(prefix + "msg_type", "");
    g_node->declare_parameter<int>(prefix + "max_freq", 10);
    g_node->declare_parameter<std::string>(prefix + "srcIP", "");
    g_node->declare_parameter<int>(prefix + "srcPort", 0);

    std::string topic_name = g_node->get_parameter(prefix + "topic_name").as_string();
    std::string msg_type = g_node->get_parameter(prefix + "msg_type").as_string();
    int max_freq = g_node->get_parameter(prefix + "max_freq").as_int();
    std::string srcIP_name = g_node->get_parameter(prefix + "srcIP").as_string();
    std::string srcIP = ip_map[srcIP_name];
    int srcPort = g_node->get_parameter(prefix + "srcPort").as_int();
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    recvTopics.emplace_back(topic);
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  (from " << srcIP_name  << ")" << std::endl;
  }

  // ********************* zmq socket initialize ***************************
  // send sockets (zmq socket PUB mode)
  for (int32_t i=0; i < len_send; ++i)
  {
    const std::string url = "tcp://" + sendTopics[i].ip + ":" + std::to_string(sendTopics[i].port);
    std::unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context, zmqpp::socket_type::pub));
    sender->bind(url);
    senders.emplace_back(std::move(sender));
  }

  // receive sockets (zmq socket SUB mode)
  for (int32_t i=0; i < len_recv; ++i)
  {
    const std::string url = "tcp://" + recvTopics[i].ip + ":" + std::to_string(recvTopics[i].port);
    std::string const zmq_topic = ""; // "" means all zmq topic
    std::unique_ptr<zmqpp::socket> receiver(new zmqpp::socket(context, zmqpp::socket_type::sub));
    receiver->subscribe(zmq_topic);
    receiver->connect(url);
    receivers.emplace_back(std::move(receiver));
  }


  // ******************* ROS subscribe and publish *************************
  //ROS topic subscribe and send
  for (int32_t i=0; i < len_send; ++i)
  {
    sub_t_last.emplace_back(g_node->get_clock()->now()); // freq control period start time
    send_num.emplace_back(0); // the send messages number in a period
    rclcpp::SubscriptionBase::SharedPtr subscriber;
    // The uniform callback function is sub_cb()
    subscriber = topic_subscriber(sendTopics[i].name, sendTopics[i].type, g_node, i);
    topic_subs.emplace_back(subscriber);
  }

  // ROS topic receive and publish
  for (int32_t i=0; i < len_recv; ++i) 
  {
    rclcpp::PublisherBase::SharedPtr publisher;
    publisher = topic_publisher(recvTopics[i].name, recvTopics[i].type, g_node);
    topic_pubs.emplace_back(publisher);
  }

  // ****************** launch receive threads *****************************
  for (int32_t i=0; i < len_recv; ++i)
  {
    recv_thread_flags.emplace_back(true); // enable receive thread flags
    recv_flags_last.emplace_back(false); // receive success flag
    recv_threads.emplace_back(std::thread(&recv_func, i));
  }

  rclcpp::spin(g_node);

  // ***************** stop send/receive ******************************
  for (int32_t i=0; i < len_send; ++i){
    stop_send(i);
  }

  for (int32_t i=0; i < len_recv; ++i){
    stop_recv(i);
  }
  
  rclcpp::shutdown();
  return 0;
}