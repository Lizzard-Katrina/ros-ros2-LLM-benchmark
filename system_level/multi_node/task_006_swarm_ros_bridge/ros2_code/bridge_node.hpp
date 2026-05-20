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
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>
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

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode();
  ~BridgeNode();

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

  template <typename T, int i>
  void sub_cb(const T &msg);

  template<typename T>
  void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i);

  std::vector<rclcpp::GenericPublisher::SharedPtr> topic_pubs;
  std::vector<rclcpp::GenericSubscription::SharedPtr> topic_subs;

  std::vector<TopicInfo> sendTopics;
  std::vector<TopicInfo> recvTopics;
  std::vector<std::unique_ptr<zmqpp::socket>> senders;
  std::vector<std::unique_ptr<zmqpp::socket>> receivers;
  zmqpp::context context;
  
  std::string ns;
  int len_send;
  int len_recv;
};

#endif