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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

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

  /* send messages frequency control */
  bool send_freq_control(int i);

  /* uniform callback functions for ROS subscribers */
  template <typename T>
  void sub_cb(const T &msg, int i);

  /* uniform deserialize and publish the receiving messages */
  template <typename T>
  void deserialize_pub(uint8_t *buffer_ptr, size_t msg_size, int i);

  /* receive thread function to receive messages and publish them */
  void recv_func(int i);

  /* close recv socket, unsubscribe ROS topic */
  void stop_send(int i);

  /* stop recv thread, close recv socket, unadvertise ROS topic */
  void stop_recv(int i);

private:
  std::vector<rclcpp::Time> sub_t_last_;
  std::vector<int> send_num_;
  std::vector<bool> recv_thread_flags_;
  std::vector<bool> recv_flags_last_;
  std::vector<std::thread> recv_threads_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> topic_subs_;
  std::vector<rclcpp::GenericPublisher::SharedPtr> topic_pubs_;
  std::vector<std::unique_ptr<zmqpp::socket>> senders_;
  std::vector<std::unique_ptr<zmqpp::socket>> receivers_;
  zmqpp::context context_;
  XmlRpc::XmlRpcValue ip_xml_;
  XmlRpc::XmlRpcValue send_topics_xml_;
  XmlRpc::XmlRpcValue recv_topics_xml_;
  std::map<std::string, std::string> ip_map_;
  std::vector<TopicInfo> sendTopics_;
  std::vector<TopicInfo> recvTopics_;
  int len_send_;
  int len_recv_;
};

#endif